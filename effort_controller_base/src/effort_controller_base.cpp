////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    effort_controller_base.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <effort_controller_base/effort_controller_base.h>
#include <cmath>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <urdf_model/joint.h>

namespace effort_controller_base
{

EffortControllerBase::EffortControllerBase()
{
}

controller_interface::InterfaceConfiguration EffortControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * m_cmd_interface_types.size());
  for (const auto& type : m_cmd_interface_types)
  {
    for (const auto & joint_name : m_joint_names)
    {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration EffortControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * m_cmd_interface_types.size());
  for (const auto& type : m_cmd_interface_types)
  {
    for (const auto & joint_name : m_joint_names)
    {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EffortControllerBase::on_init()
{
  if (!m_initialized)
  {
    auto_declare<std::string>("ik_solver", "forward_dynamics");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_base_link", "");
    auto_declare<std::string>("end_effector_link", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<double>("solver.error_scale", 1.0);
    auto_declare<int>("solver.iterations", 1);
    m_initialized = true;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EffortControllerBase::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  if (m_configured)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Load user specified inverse kinematics solver
  std::string ik_solver = get_node()->get_parameter("ik_solver").as_string();
  m_solver_loader.reset(new pluginlib::ClassLoader<IKSolver>(
    "effort_controller_base", "effort_controller_base::IKSolver"));
  try
  {
    m_ik_solver = m_solver_loader->createSharedInstance(ik_solver);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), ex.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree   robot_tree;

  m_robot_description = get_node()->get_parameter("robot_description").as_string();
  if (m_robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_robot_base_link = get_node()->get_parameter("robot_base_link").as_string();
  if (m_robot_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_end_effector_link = get_node()->get_parameter("end_effector_link").as_string();
  if (m_end_effector_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(m_robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf model from 'robot_description'");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from urdf model");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link,m_robot_chain))
  {
    const std::string error = ""
      "Failed to parse robot chain from urdf model. "
      "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(get_node()->get_logger(), error.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get names of actuated joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Initialize effort limits
  m_joint_effort_limits.resize(m_joint_names.size());

  // Parse joint limits
  KDL::JntArray upper_pos_limits(m_joint_names.size());
  KDL::JntArray lower_pos_limits(m_joint_names.size());
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    if (!robot_model.getJoint(m_joint_names[i]))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint %s does not appear in robot_description", m_joint_names[i].c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    if (robot_model.getJoint(m_joint_names[i])->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits(i) = std::nan("0");
      lower_pos_limits(i) = std::nan("0");
      m_joint_effort_limits(i) = std::nan("0");
    }
    else
    {
      // Non-existent urdf limits are zero initialized
      upper_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->upper;
      lower_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->lower;
      m_joint_effort_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->effort;
    }
  }

  // Initialize solvers
  m_ik_solver->init(get_node(),m_robot_chain,upper_pos_limits,lower_pos_limits);
  KDL::Tree tmp("not_relevant");
  tmp.addChain(m_robot_chain,"not_relevant");
  m_forward_kinematics_solver.reset(new KDL::TreeFkSolverPos_recursive(tmp));
  m_jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(m_robot_chain));
  m_iterations = get_node()->get_parameter("solver.iterations").as_int();
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();

  // Initialize Effort pd controllers
  m_spatial_controller.init(get_node());

  // Check command interfaces.
  // We support effort.
  m_cmd_interface_types = get_node()->get_parameter("command_interfaces").as_string_array();
  if (m_cmd_interface_types.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  for (const auto& type : m_cmd_interface_types)
  {
    if (type != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unsupported command interface: %s. Choose effort",
        type.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  m_configured = true;

  // Initialize effords to null
  m_efforts = ctrl::VectorND::Zero(m_joint_names.size());
  
  // Initialize joint state
  m_joint_positions.resize(m_joint_names.size());
  EffortControllerBase::updateJointStates();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EffortControllerBase::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  if (m_active)
  {
    m_joint_cmd_eff_handles.clear();
    // m_joint_cmd_pos_handles.clear();
    // m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    m_joint_state_vel_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EffortControllerBase::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  if (m_active)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Get command handles.

  if (!controller_interface::get_ordered_interfaces(command_interfaces_,
                                                    m_joint_names,
                                                    hardware_interface::HW_IF_EFFORT,
                                                    m_joint_cmd_eff_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                  "Expected %zu '%s' command interfaces, got %zu.",
                  m_joint_names.size(),
                  hardware_interface::HW_IF_EFFORT,
                  m_joint_cmd_eff_handles.size());
    return CallbackReturn::ERROR;
  }


  // Get state handles.
  // Position
  if (!controller_interface::get_ordered_interfaces(state_interfaces_,
                                                    m_joint_names,
                                                    hardware_interface::HW_IF_POSITION,
                                                    m_joint_state_pos_handles))                                                 
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(),
                 hardware_interface::HW_IF_POSITION,
                 m_joint_state_pos_handles.size());
    return CallbackReturn::ERROR;
  }

  // Velocity 
  if (!controller_interface::get_ordered_interfaces(state_interfaces_,
                                                    m_joint_names,
                                                    hardware_interface::HW_IF_VELOCITY,
                                                    m_joint_state_vel_handles))                                                 
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(),
                 hardware_interface::HW_IF_VELOCITY,
                 m_joint_state_vel_handles.size());
    return CallbackReturn::ERROR;
  }



  

  // Copy joint state to internal simulation
  // if (!m_ik_solver->setStartState(m_joint_state_pos_handles))
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Could not set start state");
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  // };
  // m_ik_solver->updateKinematics();

  // Provide safe command buffers with starting where we are
  computeJointEffortCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  writeJointEffortCmds();

  m_active = true;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EffortControllerBase::writeJointEffortCmds()
{
  // Write all available types.
  for (const auto& type : m_cmd_interface_types)
  {
    if (type == hardware_interface::HW_IF_EFFORT)
    {
      for (size_t i = 0; i < m_joint_names.size(); ++i)
      {
        // Effort saturation
        if (m_efforts[i] >= m_joint_effort_limits(i))
        {
          m_efforts[i] = m_joint_effort_limits(i);
        }
        else if (m_efforts[i] <= -m_joint_effort_limits(i))
        {
          m_efforts[i] = -m_joint_effort_limits(i);
        }
        m_joint_cmd_eff_handles[i].get().set_value(m_efforts[i]);
      }
    }
  }
}

void EffortControllerBase::computeJointEffortCmds(const ctrl::Vector6D& error, const rclcpp::Duration& period)
{

  m_jnt_to_jac_solver->JntToJac(m_joint_positions, m_jacobian);
  m_efforts = m_jacobian.data.transpose() * error;
  // // PD controlled system input
  // m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();
  // m_effort_input = m_error_scale * m_spatial_controller(error,period);

  // // Simulate one step forward
  // m_simulated_joint_motion = m_ik_solver->getJointControlCmds(
  //     period,
  //     m_effort_input);

  // m_ik_solver->updateKinematics();
}

ctrl::Vector6D EffortControllerBase::displayInBaseLink(const ctrl::Vector6D& vector, const std::string& from)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(
      m_ik_solver->getPositions(),
      transform_kdl,
      from);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

ctrl::Matrix6D EffortControllerBase::displayInBaseLink(const ctrl::Matrix6D& tensor, const std::string& from)
{
  // Get rotation to base
  KDL::Frame R_kdl;
  m_forward_kinematics_solver->JntToCart(
      m_ik_solver->getPositions(),
      R_kdl,
      from);

  // Adjust format
  ctrl::Matrix3D R;
  R <<
      R_kdl.M.data[0],
      R_kdl.M.data[1],
      R_kdl.M.data[2],
      R_kdl.M.data[3],
      R_kdl.M.data[4],
      R_kdl.M.data[5],
      R_kdl.M.data[6],
      R_kdl.M.data[7],
      R_kdl.M.data[8];

  // Treat diagonal blocks as individual 2nd rank tensors.
  // Display in base frame.
  ctrl::Matrix6D tmp = ctrl::Matrix6D::Zero();
  tmp.topLeftCorner<3,3>() = R * tensor.topLeftCorner<3,3>() * R.transpose();
  tmp.bottomRightCorner<3,3>() = R * tensor.bottomRightCorner<3,3>() * R.transpose();

  return tmp;
}

ctrl::Vector6D EffortControllerBase::displayInTipLink(const ctrl::Vector6D& vector, const std::string& to)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(
      m_ik_solver->getPositions(),
      transform_kdl,
      to);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M.Inverse() * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

void EffortControllerBase::updateJointStates(){
  for (auto i = 0; i < (int)m_joint_names.size(); ++i) {
    const auto& position_interface = m_joint_state_pos_handles[i].get();

    m_joint_positions(i) = position_interface.get_value();
  }
}

} // namespace
