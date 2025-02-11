#ifndef PD_CONTROLLER_FRANKA_PANDA_WITH_COMPENSATION_H_INCLUDED
#define PD_CONTROLLER_FRANKA_PANDA_WITH_COMPENSATION_H_INCLUDED

#include <vector>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <control_msgs/msg/multi_dof_command.hpp>

#include <franka_semantic_components/franka_robot_model.hpp>

#include <Eigen/Eigen>

using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace pd_controller_franka_panda_with_compensation
{
class PDFrankaPandaControllerCompensation : public controller_interface::ControllerInterface
{
public:
  virtual ~PDFrankaPandaControllerCompensation() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  void ref_state_callback(const control_msgs::msg::MultiDOFCommand::SharedPtr ref_state);

  using ControllerReferenceMsg = control_msgs::msg::MultiDOFCommand;

private:
  size_t m_num_joints_;
  std::string m_arm_id_;
  std::vector<std::string> m_joint_names_;
  std::vector<std::string> m_state_interfaces_names_;
  std::string m_command_interface_name_;
  bool rec;

  // eigen vectors
  Vector7d q_;
  Vector7d dq_;
  Vector7d initial_q_;
  Vector7d q_goal;
  Vector7d q_dot_goal;
  Vector7d tau_d_calculated;
  Vector7d dq_filtered_;
  Vector7d m_p_gain_val_;
  Vector7d m_d_gain_val_;
  // reference subscriber and buffer
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  // robot_model
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  void updateJointStates();
};
}  // namespace pd_controller_franka_panda_with_compensation
#endif  // PD_CONTROLLER_FRANKA_PANDA_WITH_COMPENSATION_H_INCLUDED