#include <pd_controller_franka_panda_with_compensation/pd_controller_franka_panda_with_compensation.h>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace pd_controller_franka_panda_with_compensation {

controller_interface::InterfaceConfiguration
PDFrankaPandaControllerCompensation::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= m_num_joints_; ++i)
  {
    config.names.push_back(m_arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
PDFrankaPandaControllerCompensation::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= m_num_joints_; ++i)
  {
    config.names.push_back(m_arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(m_arm_id_ + "_joint" + std::to_string(i) + "/velocity");

    if (!simulation)
    {
      for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names())
      {
        config.names.push_back(franka_robot_model_name);
      }
    }
  }
  return config;
}

controller_interface::CallbackReturn PDFrankaPandaControllerCompensation::on_init()
{
  try
  {
    rec = false;

    auto_declare<std::string>("arm_id", "");
    auto_declare<bool>("simulation", true);
    auto_declare<std::vector<double> >("p_gains", {});
    auto_declare<std::vector<double> >("d_gains", {});
    // do we need robot description?
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PDFrankaPandaControllerCompensation::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // assign parameters
  m_num_joints_ = 7;
  dq_filtered_.setZero();
  m_arm_id_   = get_node()->get_parameter("arm_id").as_string();
  simulation = get_node()->get_parameter("simulation").as_bool();
  auto p_gain = get_node()->get_parameter("p_gains").as_double_array();
  auto d_gain = get_node()->get_parameter("d_gains").as_double_array();

  if (p_gain.empty())
  {
    RCLCPP_FATAL(get_node()->get_logger(), "m_p_gain_val_ parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (p_gain.size() != static_cast<uint>(m_num_joints_))
  {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "m_p_gain_val_ should be of size %d but is of size %ld",
                 m_num_joints_,
                 m_p_gain_val_.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gain.empty())
  {
    RCLCPP_FATAL(get_node()->get_logger(), "m_d_gain_val_ parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gain.size() != static_cast<uint>(m_num_joints_))
  {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "m_d_gain_val_ should be of size %d but is of size %ld",
                 m_num_joints_,
                 m_d_gain_val_.size());
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < m_num_joints_; ++i)
  {
    m_p_gain_val_(i) = p_gain.at(i);
    m_d_gain_val_(i) = d_gain.at(i);
  }

  // robot model
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
    franka_semantic_components::FrankaRobotModel(m_arm_id_ + "/" + "robot_state",
                                                 m_arm_id_ + "/" + "robot_model"));

  home_pos = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/home_pos",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      home_pos_cb(request, response);
    });


  // reference state subscriber
  ref_subscriber_ = get_node()->create_subscription<control_msgs::msg::MultiDOFCommand>(
    m_arm_id_ + "/reference",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &PDFrankaPandaControllerCompensation::ref_state_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PDFrankaPandaControllerCompensation::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  updateJointStates();
  initial_q_ = q_;
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDFrankaPandaControllerCompensation::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
PDFrankaPandaControllerCompensation::update(const rclcpp::Time& /*time*/,
                                            const rclcpp::Duration& period)
{
  updateJointStates();

  // implement as an action similar to joint_velocity_change
  if (rec)
  {
    for (int i = 0; i < m_num_joints_; ++i)
    {
      q_goal(i)     = input_ref_.readFromNonRT()->get()->values.at(i);
      q_dot_goal(i) = input_ref_.readFromNonRT()->get()->values_dot.at(i);

      if (!simulation)
      {
        cor_comp_(i)     = franka_robot_model_->getCoriolisForceVector()[i];
        gravity_comp_(i) = franka_robot_model_->getGravityForceVector()[i];
      }
    }

    tau_d_calculated =
      m_p_gain_val_.cwiseProduct(q_goal - q_) + m_d_gain_val_.cwiseProduct(q_dot_goal - dq_);
    if (!simulation)
    {
      tau_d_calculated += gravity_comp_ + cor_comp_.cwiseProduct(dq_);
    }
  }
  else
  {
    q_goal              = initial_q_;
    const double kAlpha = 0.99;
    dq_filtered_        = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    tau_d_calculated =
      m_p_gain_val_.cwiseProduct(q_goal - q_) + m_d_gain_val_.cwiseProduct(-dq_filtered_);
  }

  for (int i = 0; i < m_num_joints_; ++i)
  {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

void PDFrankaPandaControllerCompensation::ref_state_callback(
  const control_msgs::msg::MultiDOFCommand::SharedPtr ref_state)
{
  (void)ref_state;
  input_ref_.writeFromNonRT(ref_state);
  rec = true;
}

void PDFrankaPandaControllerCompensation::updateJointStates()
{
  for (auto i = 0; i < m_num_joints_; ++i)
  {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i)  = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void PDFrankaPandaControllerCompensation::home_pos_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  rec               = false;
  response->success = true;
}

} // namespace pd_controller_franka_panda_with_compensation


#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  pd_controller_franka_panda_with_compensation::PDFrankaPandaControllerCompensation,
  controller_interface::ControllerInterface)