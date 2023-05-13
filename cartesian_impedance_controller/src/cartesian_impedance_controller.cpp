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
/*!\file    cartesian_impedance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "effort_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <cartesian_impedance_controller/cartesian_impedance_controller.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace cartesian_impedance_controller
{

CartesianImpedanceController::CartesianImpedanceController()
: Base::EffortControllerBase(), m_hand_frame_control(true)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if(!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link << " is not part of the kinematic chain from "
                                             << Base::m_robot_base_link << " to "
                                             << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"),
    10,
    std::bind(&CartesianImpedanceController::targetWrenchCallback, this, std::placeholders::_1));

  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_node()->get_name() + std::string("/ft_sensor_wrench"),
      10,
      std::bind(&CartesianImpedanceController::ftSensorWrenchCallback, this, std::placeholders::_1));

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Stop drifting by sending zero joint velocities
  Base::computeJointControlCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  Base::writeJointControlCmds();
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
{
  // // Synchronize the internal model and the real robot
  // Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // // Control the robot motion in such a way that the resulting net force
  // // vanishes.  The internal 'simulation time' is deliberately independent of
  // // the outer control cycle.
  // auto internal_period = rclcpp::Duration::from_seconds(0.02);

  // // Compute the net force
  // ctrl::Vector6D error = computeForceError();

  // // Turn Effort error into joint motion
  // Base::computeJointControlCmds(error,internal_period);

  // // Write final commands to the hardware interface
  // Base::writeJointControlCmds();

  // Extract the current joint positions

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(m_joint_positions, m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::Matrix6D J_pinv;
  Eigen::Map<Eigen::MatrixXd> J_eigen(J_kdl.data, J_kdl.rows(), J_kdl.columns());
  pseudoInverse(m_jacobian.transpose(), J_pinv.data);
  // Eigen::Map<Eigen::MatrixXd> J_eigen(J_kdl.data, J_kdl.rows(), J_kdl.columns());
  
  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianImpedanceController::computeForceError()
{
  ctrl::Vector6D target_wrench;
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control) // Assume end-effector frame by convention
  {
    target_wrench = Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link);
  }
  else // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench,m_new_ft_sensor_ref) + target_wrench;
}

ctrl::Vector6D CartesianImpedanceController::computeMotionError()
{
  // Compute motion error wrt robot_base_link
  KDL::Frame m_current_frame;
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Transformation from target -> current corresponds to error = target - current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle    = error_kdl.M.GetRotAngle(rot_axis);   // rot_axis is normalized

  rot_axis = rot_axis * angle;

  // Reassign values
  ctrl::Vector6D error;
  error(0) = error_kdl.p.x();
  error(1) = error_kdl.p.y();
  error(2) = error_kdl.p.z();
  error(3) = rot_axis(0);
  error(4) = rot_axis(1);
  error(5) = rot_axis(2);

  return error;
}
// void CartesianImpedanceController::setFtSensorReferenceFrame(const std::string& new_ref)
// {
//   // Compute static transform from the force torque sensor to the new reference
//   // frame of interest.
//   m_new_ft_sensor_ref = new_ref;

//   // Joint positions should cancel out, i.e. it doesn't matter as long as they
//   // are the same for both transformations.
//   KDL::JntArray jnts(Base::m_ik_solver->getPositions());

//   KDL::Frame sensor_ref;
//   Base::m_forward_kinematics_solver->JntToCart(
//       jnts,
//       sensor_ref,
//       m_ft_sensor_ref_link);

//   KDL::Frame new_sensor_ref;
//   Base::m_forward_kinematics_solver->JntToCart(
//       jnts,
//       new_sensor_ref,
//       m_new_ft_sensor_ref);

//   m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
// }

void CartesianImpedanceController::targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianImpedanceController::targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target)
{
  if (target->header.frame_id != Base::m_robot_base_link)
  {
    auto& clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
        clock, 3000,
        "Got target pose in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(),
        target->header.frame_id.c_str());
    return;
  }

  m_target_frame = KDL::Frame(
      KDL::Rotation::Quaternion(
        target->pose.orientation.x,
        target->pose.orientation.y,
        target->pose.orientation.z,
        target->pose.orientation.w),
      KDL::Vector(
        target->pose.position.x,
        target->pose.position.y,
        target->pose.position.z));
}
// void CartesianImpedanceController::ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
// {
//   KDL::Wrench tmp;
//   tmp[0] = wrench->wrench.force.x;
//   tmp[1] = wrench->wrench.force.y;
//   tmp[2] = wrench->wrench.force.z;
//   tmp[3] = wrench->wrench.torque.x;
//   tmp[4] = wrench->wrench.torque.y;
//   tmp[5] = wrench->wrench.torque.z;

//   // Compute how the measured wrench appears in the frame of interest.
//   tmp = m_ft_sensor_transform * tmp;

//   m_ft_sensor_wrench[0] = tmp[0];
//   m_ft_sensor_wrench[1] = tmp[1];
//   m_ft_sensor_wrench[2] = tmp[2];
//   m_ft_sensor_wrench[3] = tmp[3];
//   m_ft_sensor_wrench[4] = tmp[4];
//   m_ft_sensor_wrench[5] = tmp[5];
// }

}

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceController, controller_interface::ControllerInterface)