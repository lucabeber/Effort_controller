#ifndef EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <effort_controller_base/effort_controller_base.h>
#include <controller_interface/controller_interface.hpp>

namespace cartesian_impedance_controller
{

/**
 * @brief A ROS2-control controller for Effort force control
 *
 * This controller implements 6-dimensional end effector force control for
 * robots with a wrist force-torque sensor.  Users command
 * geometry_msgs::msg::WrenchStamped targets to steer the robot in task space.  The
 * controller additionally listens to the specified force-torque sensor signals
 * and computes the superposition with the target wrench.
 *
 * The underlying solver maps this remaining wrench to joint motion.
 * Users can steer their robot with this control in free space. The speed of
 * the end effector motion is set with PD gains on each Effort axes.
 * In contact, the controller regulates the net force of the two wrenches to zero.
 *
 * Note that during free motion, users can generally set higher control gains
 * for faster motion.  In contact with the environment, however, normally lower
 * gains are required to maintain stability.  The ranges to operate in mainly
 * depend on the stiffness of the environment and the controller cycle of the
 * real hardware, such that some experiments might be required for each use
 * case.
 *
 */
class CartesianImpedanceController : public virtual effort_controller_base::EffortControllerBase
{
  public:
    CartesianImpedanceController();

    virtual LifecycleNodeInterface::CallbackReturn on_init() override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    ctrl::VectorND computeTorque();

    using Base = effort_controller_base::EffortControllerBase;

    ctrl::Matrix6D          m_cartesian_stiffness;
    ctrl::Matrix6D          m_cartesian_damping;
    double                  m_null_space_stiffness; 
    double                  m_null_space_damping; 
    ctrl::Vector6D          m_target_wrench;

  protected:
    /**
     * @brief Compute the net force of target wrench and measured sensor wrench
     *
     * @return The remaining error wrench, given in robot base frame
     */
    ctrl::Vector6D        computeForceError();
    std::string           m_new_ft_sensor_ref;
    void setFtSensorReferenceFrame(const std::string& new_ref);

  private:
    ctrl::Vector6D        compensateGravity();

    void targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
    void targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target);
    ctrl::Vector6D computeMotionError();

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_target_frame_subscriber;
    KDL::Frame            m_target_frame;
    ctrl::Vector6D        m_ft_sensor_wrench;
    std::string           m_ft_sensor_ref_link;
    KDL::Frame            m_ft_sensor_transform;

    KDL::JntArray         m_null_space;
    KDL::Frame            m_current_frame;

    ctrl::MatrixND        m_identity;
    ctrl::VectorND        m_q_starting_pose; 
    ctrl::VectorND        m_tau_old;

    ctrl::Vector3D        m_old_rot_error;
    ctrl::VectorND        m_old_vel_error;
    double const m_alpha = 0.3;
    /**
     * Allow users to choose whether to specify their target wrenches in the
     * end-effector frame (= True) or the base frame (= False). The first one
     * is easier for explicit task programming, while the second one is more
     * intuitive for tele-manipulation.
     */
    bool m_hand_frame_control;

    bool m_with_postural_task;

};

}

#endif
