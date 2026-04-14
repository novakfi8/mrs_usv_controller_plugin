/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <pid.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace mrs_usv_controller_plugin
{

namespace position_controller
{

/* DrsParams_t //{ */

struct DrsParams_t
{
  bool debug;
  bool stop_publishing;
};

//}

/* //{ class PositionController */

class PositionController : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput &last_control_output);

  void deactivate(void);

  void destroy(void);

  void updateInactive(const mrs_msgs::msg::UavState &uav_state, const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command);

  ControlOutput updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command);

  const mrs_msgs::msg::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::msg::UavState &new_uav_state);

  void resetDisturbanceEstimators(void);

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
  setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  std::string node_name = "MrsUsvController_PositionController";
  
  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::msg::UavState uav_state_;
  std::mutex              mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |
 
  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
  std::mutex                            mutex_dynparam_mgr_;
  DrsParams_t                           drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::msg::DynamicsConstraints constraints_;
  std::mutex                         mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  rclcpp::Time      last_update_time_;
  std::atomic<bool> first_iteration_ = true;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool PositionController::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_  = node;
  clock_ = node_->get_clock();

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = clock_->now();

  // | ------------------- loading parameters ------------------- |
  
  // add yaml config from the package's config folder
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("mrs_usv_controller_plugin") + "/config/position_controller.yaml");

  // setup the dynamical reconfigure parameter manager
  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_dynparam_mgr_);  
  dynparam_mgr_->get_param_provider().copyYamls(private_handlers->param_loader->getParamProvider());

  // setup dynamically-reconfigurable parameters
  dynparam_mgr_->register_param("debug", &drs_params_.debug);
  dynparam_mgr_->register_param("stop_publishing", &drs_params_.stop_publishing);

  // | ------------------ finish loading params ----------------- |

  if (!private_handlers->param_loader->loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool PositionController::activate(const ControlOutput &last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;
  is_active_ = true;

  RCLCPP_INFO(node_->get_logger(), "activated");

  return true;
}

//}

/* //{ deactivate() */

void PositionController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "deactivated");
}

//}

/* //{ destroy() */

void PositionController::destroy(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "destroyed");
}

//}

/* updateInactive() //{ */

void PositionController::updateInactive(const mrs_msgs::msg::UavState &uav_state, [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

PositionController::ControlOutput PositionController::updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command) {

  auto drs_params = mrs_lib::get_mutexed(mutex_dynparam_mgr_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  // clear all the optional parts of the result
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  if (!is_active_) {
    return last_control_output_;
  }

  // | -------- check for the available output modalities ------- |

  if (!common_handlers_->control_output_modalities.position) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "desired position output modality is not available");
    drs_params.stop_publishing = true; 
  }

  // | ---------------- prepare the control output --------------- |

  mrs_msgs::msg::HwApiPositionCmd position_cmd;

  position_cmd.position.x = tracker_command.position.x;
  position_cmd.position.y = tracker_command.position.y;

  // | ----------------- set the control output ----------------- |

  position_cmd.header = tracker_command.header; 
  last_control_output_.control_output = position_cmd;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "PositionController";
 
  if(drs_params.debug) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000,"position_cmd: [%.3f, %.3f, %.3f]", position_cmd.position.x, position_cmd.position.y, position_cmd.position.z);
  }

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::msg::ControllerStatus PositionController::getStatus() {

  mrs_msgs::msg::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void PositionController::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void PositionController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
PositionController::setConstraints([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) {

  if (!is_initialized_) {
    return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  RCLCPP_INFO(node_->get_logger(), "updating constraints");

  mrs_msgs::srv::DynamicsConstraintsSrv::Response res;
  res.success = true;
  res.message = "constraints updated";

  return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response(res));
}

//}

}  // namespace position_controller

}  // namespace mrs_usv_controller_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_usv_controller_plugin::position_controller::PositionController, mrs_uav_managers::Controller)
