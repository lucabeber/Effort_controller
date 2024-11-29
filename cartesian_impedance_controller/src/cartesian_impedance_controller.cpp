#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <cartesian_impedance_controller/pseudo_inversion.h>

#include "controller_interface/controller_interface.hpp"
#include "effort_controller_base/Utility.h"

namespace cartesian_impedance_controller {

CartesianImpedanceController::CartesianImpedanceController()
    : Base::EffortControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);
  auto_declare<bool>("postural_task", true);
  auto_declare<double>("nullspace_stiffness", 10);

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
  ;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link =
      get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link
                            << " is not part of the kinematic chain from "
                            << Base::m_robot_base_link << " to "
                            << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }

  // Set stiffness
  ctrl::Vector6D tmp;
  tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  m_cartesian_stiffness = tmp.asDiagonal();

  // Set damping
  tmp[0] = 2 * sqrt(tmp[0]);
  tmp[1] = 2 * sqrt(tmp[1]);
  tmp[2] = 2 * sqrt(tmp[2]);
  tmp[3] = 2 * sqrt(tmp[3]);
  tmp[4] = 2 * sqrt(tmp[4]);
  tmp[5] = 2 * sqrt(tmp[5]);

  m_cartesian_damping = tmp.asDiagonal();

  // Set nullspace stiffness
  m_null_space_stiffness = get_node()->get_parameter("nullspace_stiffness").as_double();
  RCLCPP_INFO(get_node()->get_logger(), "Postural task stiffness: %f", m_null_space_stiffness);

  // Set nullspace damping
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

  // Set the identity matrix with dimension of the joint space
  m_identity = ctrl::MatrixND::Identity(m_joint_number, m_joint_number);

  // Make sure sensor wrenches are interpreted correctly
  // setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/target_wrench"), 10,
          std::bind(&CartesianImpedanceController::targetWrenchCallback, this,
                    std::placeholders::_1));

  // m_ft_sensor_wrench_subscriber =
  //   get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     get_node()->get_name() + std::string("/ft_sensor_wrench"),
  //     10,
  //     std::bind(&CartesianImpedanceController::ftSensorWrenchCallback, this,
  //     std::placeholders::_1));

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 3,
          std::bind(&CartesianImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));

  m_with_postural_task = get_node()->get_parameter("postural_task").as_bool();
  if(m_with_postural_task) {
    RCLCPP_INFO(get_node()->get_logger(), "Postural task is activated");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Postural task is NOT activated");
  }

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set the target frame to the current frame
  m_target_frame = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  m_q_starting_pose = Base::m_joint_positions.data;

  // Initialize the old torque to zero
  m_tau_old = ctrl::VectorND::Zero(Base::m_joint_number);

  m_old_rot_error = ctrl::Vector3D::Zero();

  m_old_vel_error = ctrl::VectorND::Zero(Base::m_joint_number);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  Base::writeJointEffortCmds();
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Update joint states
  Base::updateJointStates();

  // Compute the torque to applay at the joints
  ctrl::VectorND tau_tot = computeTorque();

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianImpedanceController::computeForceError() {
  ctrl::Vector6D target_wrench;
  m_hand_frame_control =
      get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control)  // Assume end-effector frame by convention
  {
    target_wrench =
        Base::displayInBaseLink(m_target_wrench, Base::m_end_effector_link);
  } else  // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench, m_new_ft_sensor_ref) +
         target_wrench;
}

ctrl::Vector6D CartesianImpedanceController::computeMotionError() {
  // Compute the cartesian error between the current and the target frame
  ctrl::Vector6D error;

  // Compute the difference between the two frames
  KDL::Twist error_frame;

  // Compute the error frame
  error_frame.vel = m_current_frame.p - m_target_frame.p;
  error_frame.rot =
      0.5 * (m_target_frame.M.UnitX() * m_current_frame.M.UnitX() +
             m_target_frame.M.UnitY() * m_current_frame.M.UnitY() +
             m_target_frame.M.UnitZ() * m_current_frame.M.UnitZ());

  // Compute the error vector
  error(0) = error_frame(0);
  error(1) = error_frame(1);
  error(2) = error_frame(2);
  error(3) = error_frame(3);//m_alpha * error_frame(3) + (1 - m_alpha) * m_old_rot_error(0);
  error(4) = error_frame(4);//m_alpha * error_frame(4) + (1 - m_alpha) * m_old_rot_error(1);
  error(5) = error_frame(5);//m_alpha * error_frame(5) + (1 - m_alpha) * m_old_rot_error(2);

  // error(3) = std::round(error(3) * 1000) / 1000;
  // error(4) = std::round(error(4) * 1000) / 1000;
  // error(5) = std::round(error(5) * 1000) / 1000;

  // // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // // 100,
  // //                      "rot error: %f %f %f ", error(3), error(4), error(5));

  // m_old_rot_error(0) = error(3);
  // m_old_rot_error(1) = error(4);
  // m_old_rot_error(2) = error(5);

  // Print the target frame, the current frame and the error frame
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "target frame: " <<
  // m_target_frame.p.x() << " " << m_target_frame.p.y() << " " <<
  // m_target_frame.p.z() << " " << m_target_frame.M.GetRot().x() << " " <<
  // m_target_frame.M.GetRot().y() << " " << m_target_frame.M.GetRot().z());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "current frame: " <<
  // m_current_frame.p.x() << " " << m_current_frame.p.y() << " " <<
  // m_current_frame.p.z() << " " << m_current_frame.M.GetRot().x() << " " <<
  // m_current_frame.M.GetRot().y() << " " << m_current_frame.M.GetRot().z());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "error frame: " <<
  // error_frame(0) << " " << error_frame(1) << " " << error_frame(2) << " " <<
  // error_frame(3) << " " << error_frame(4) << " " << error_frame(5));
  return error;
}

ctrl::VectorND CartesianImpedanceController::computeTorque() {
  // Find the desired joints positions
  // Base::computeNullSpace(m_target_frame);

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions,
                                      Base::m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::MatrixND jac = Base::m_jacobian.data;
  ctrl::MatrixND jac_tran_pseudo_inverse;

  pseudoInverse(jac.transpose(), &jac_tran_pseudo_inverse);

  // Redefine joints velocities in Eigen format
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  ctrl::VectorND q_null_space = Base::m_simulated_joint_motion.data;

  // Compute the motion error
  ctrl::Vector6D motion_error = computeMotionError();

  ctrl::VectorND tau_task(Base::m_joint_number), tau_null(Base::m_joint_number),
      tau_ext(Base::m_joint_number);

  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // 500, "motion_error: %f %f %f %f %f %f", motion_error(0), motion_error(1),
  // motion_error(2), motion_error(3), motion_error(4), motion_error(5)); Torque
  // calculation for task space

  q_dot = m_alpha * q_dot + (1 - m_alpha) * m_old_vel_error;

  q_dot(0) = std::round(q_dot(0) * 1000) / 1000;
  q_dot(1) = std::round(q_dot(1) * 1000) / 1000;
  q_dot(2) = std::round(q_dot(2) * 1000) / 1000;
  q_dot(3) = std::round(q_dot(3) * 1000) / 1000;
  q_dot(4) = std::round(q_dot(4) * 1000) / 1000;
  q_dot(5) = std::round(q_dot(5) * 1000) / 1000;
  q_dot(6) = std::round(q_dot(6) * 1000) / 1000;

  m_old_vel_error = q_dot;

  tau_task = jac.transpose() * (- m_cartesian_stiffness * motion_error -
             (m_cartesian_damping * (jac * q_dot)));

  // print damping
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                       "q_dot: %f %f %f %f %f %f %f", q_dot(0), q_dot(1),
                       q_dot(2), q_dot(3), q_dot(4), q_dot(5), q_dot(6));

  q_null_space = m_q_starting_pose;
  tau_null = (m_identity - jac.transpose() * jac_tran_pseudo_inverse) *
             (m_null_space_stiffness * (-q + q_null_space) -
              m_null_space_damping * q_dot);

  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // 500, "tau_null: %f %f %f %f %f %f %f", tau_null(0), tau_null(1),
  // tau_null(2), tau_null(3), tau_null(4), tau_null(5), tau_null(6));
  // // Torque calculation for external wrench
  // tau_ext = jac.transpose() * m_target_wrench;

  // // Final torque calculation
  // return - tau_task + tau_null + tau_ext;

  KDL::JntArray gravity, coriolis;
  gravity.resize(Base::m_joint_number);
  coriolis.resize(Base::m_joint_number);

  // print joint position
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // 500,
  //                      "joint position: %f %f %f %f %f %f %f",
  //                      Base::m_joint_positions(0),
  //                      Base::m_joint_positions(1),
  //                      Base::m_joint_positions(2),
  //                      Base::m_joint_positions(3),
  //                      Base::m_joint_positions(4),
  //                      Base::m_joint_positions(5),
  //                      Base::m_joint_positions(6));
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // 500,
  //                      "q_null_space: %f %f %f %f %f %f %f", q_null_space(0),
  //                      q_null_space(1), q_null_space(2), q_null_space(3),
  //                      q_null_space(4), q_null_space(5), q_null_space(6));

  // print joint torque
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // 500,
  //                      "joint torque: %f %f %f %f %f %f %f", tau_task(0),

  //                      tau_task(1), tau_task(2), tau_task(3), tau_task(4),
  //                      tau_task(5), tau_task(6));

  // RCLCPP_INFO_THROTTLE(
  //     get_node()->get_logger(), *get_node()->get_clock(), 500,
  //     "null error %f %f %f %f %f %f %f", q_null_space(0) - q(0),
  //     q_null_space(1) - q(1), q_null_space(2) - q(2), q_null_space(3) - q(3),
  //     q_null_space(4) - q(4), q_null_space(5) - q(5), q_null_space(6) -
  //     q(6));

  // Base::m_dyn_solver->JntToGravity(Base::m_joint_positions,gravity);
  // Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions,
  // Base::m_joint_velocities,coriolis);
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // 500, "gravity: %f %f %f %f %f %f", gravity(0), gravity(1), gravity(2),
  // gravity(3), gravity(4), gravity(5));
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
  // 500, "coriolis: %f %f %f %f %f %f %f", coriolis(0), coriolis(1),
  // coriolis(2), coriolis(3), coriolis(4), coriolis(5), coriolis(6));
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "tau_task: " << tau_task);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "gravity: " << gravity.data);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "coriolis: " <<
  // coriolis.data);
  // ctrl::VectorND tau_tot(Base::m_joint_number);
  // tau_tot = 0.5 * tau_task + 0.5 * m_tau_old;
  // m_tau_old = tau_task;

  return tau_task + m_with_postural_task * tau_null;  // + coriolis.data;
}
// void CartesianImpedanceController::setFtSensorReferenceFrame(const
// std::string& new_ref)
// {
//   // Compute static transform from the force torque sensor to the new
//   reference
//   // frame of interest.
//   m_new_ft_sensor_ref = new_ref;

//   // Joint positions should cancel out, i.e. it doesn't matter as long as
//   they
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

void CartesianImpedanceController::targetWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianImpedanceController::targetFrameCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr target) {
  if (target->header.frame_id != Base::m_robot_base_link)
  {
    auto& clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "Got target pose in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  m_target_frame =
      KDL::Frame(KDL::Rotation::Quaternion(
                     target->pose.orientation.x, target->pose.orientation.y,
                     target->pose.orientation.z, target->pose.orientation.w),
                 KDL::Vector(target->pose.position.x, target->pose.position.y,
                             target->pose.position.z));
}
// void CartesianImpedanceController::ftSensorWrenchCallback(const
// geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
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

}  // namespace cartesian_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    cartesian_impedance_controller::CartesianImpedanceController,
    controller_interface::ControllerInterface)
