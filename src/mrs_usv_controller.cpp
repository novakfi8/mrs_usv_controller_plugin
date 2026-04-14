/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/dynparam_mgr.h>

#include <pid.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
//}

namespace mrs_usv_controller_plugin
{

namespace mrs_usv_controller
{

/* Enumeration to address controller state //{ */
namespace PidController
{
  enum states
  {
    idle = 0,
    only_heading = 1,
    heading_and_velocity = 2,
  };
}
//}

/* DrsParams_t //{ */

struct DrsParams_t
{
  bool debug;
  bool publish_zero_outputs;

  bool   enforce_constraints;
  bool   desired_heading_to_final_reference;
  double max_cmd_velocity;
  double max_cmd_heading_rate;
  double horizontal_speed_constraint;
  double vertical_asc_speed_constraint;
  
  double low_position_error;
  double max_position_error;
  double low_heading_error;
  double max_heading_error;

  double gain_position_p;
  double gain_position_i;
  double gain_position_d;
  double saturation_position;
  double antiwindup_position;
  
  double gain_heading_rate_p;
  double gain_heading_rate_i;
  double gain_heading_rate_d;
  double saturation_heading_rate;
  double antiwindup_heading_rate;
};

//}

/* //{ class MrsUsvController */

class MrsUsvController : public mrs_uav_managers::Controller {

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

  std::string node_name = "MrsUsvController";

  bool is_initialized_ = false;
  bool is_active_      = false;

  double last_heading_desired_ = 0;

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

  // | ----------------------- controller ----------------------- |
  PIDController pid_controller_heading_rate_;
  std::mutex mutex_heading_rate_pid_;
  
  PIDController pid_controller_position_;
  std::mutex mutex_position_pid_;
  
  int pid_state_;
  std::mutex mutex_pid_state_;
  
  // | -------------------------- utils ------------------------- |
  int  stateMachineFunction(int pid_state, double heading_error, double position_error, DrsParams_t drs_params);
  double getHeading(double x1, double y1, double x2, double y2, double rescue_heading);
  std::string getHeadingRatePidStateName(int heading_rate_pid_state);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool MrsUsvController::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_  = node;
  clock_ = node_->get_clock();

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = clock_->now();

  // | ------------------- loading parameters ------------------- |
  
  // add yaml config from the package's config folder
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("mrs_usv_controller_plugin") + "/config/mrs_usv_controller.yaml");

  // setup the dynamical reconfigure parameter manager
  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_dynparam_mgr_);  
  dynparam_mgr_->get_param_provider().copyYamls(private_handlers->param_loader->getParamProvider());

  // setup dynamically-reconfigurable parameters
  dynparam_mgr_->register_param("debug", &drs_params_.debug);
  dynparam_mgr_->register_param("publish_zero_outputs", &drs_params_.publish_zero_outputs);
  
  dynparam_mgr_->register_param("controller/enforce_constraints", &drs_params_.enforce_constraints);
  dynparam_mgr_->register_param("controller/desired_heading_to_final_reference", &drs_params_.desired_heading_to_final_reference);
  dynparam_mgr_->register_param("controller/max_cmd_velocity", &drs_params_.max_cmd_velocity, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  dynparam_mgr_->register_param("controller/max_cmd_heading_rate", &drs_params_.max_cmd_heading_rate, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  dynparam_mgr_->register_param("controller/horizontal_speed_constraint", &drs_params_.horizontal_speed_constraint, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  dynparam_mgr_->register_param("controller/vertical_asc_speed_constraint", &drs_params_.vertical_asc_speed_constraint, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  
  dynparam_mgr_->register_param("state_machine/low_position_error", &drs_params_.low_position_error, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  dynparam_mgr_->register_param("state_machine/max_position_error", &drs_params_.max_position_error, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  dynparam_mgr_->register_param("state_machine/low_heading_error", &drs_params_.low_heading_error, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  dynparam_mgr_->register_param("state_machine/max_heading_error", &drs_params_.max_heading_error, mrs_lib::DynparamMgr::range_t<double>(0.0, 10.0));
  
  dynparam_mgr_->register_param("pid_position/gain_position_p", &drs_params_.gain_position_p, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_position/gain_position_i", &drs_params_.gain_position_i, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_position/gain_position_d", &drs_params_.gain_position_d, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_position/saturation_position", &drs_params_.saturation_position, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_position/antiwindup_position", &drs_params_.antiwindup_position, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  
  dynparam_mgr_->register_param("pid_heading/gain_heading_rate_p", &drs_params_.gain_heading_rate_p, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_heading/gain_heading_rate_i", &drs_params_.gain_heading_rate_i, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_heading/gain_heading_rate_d", &drs_params_.gain_heading_rate_d, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_heading/saturation_heading_rate", &drs_params_.saturation_heading_rate, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_heading/antiwindup_heading_rate", &drs_params_.antiwindup_heading_rate, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));

  // | ----------------------- prepare PID ---------------------- |

  pid_controller_position_.setParams(drs_params_.gain_position_p, drs_params_.gain_position_i, drs_params_.gain_position_d, drs_params_.saturation_position, drs_params_.antiwindup_position);
  pid_controller_heading_rate_.setParams(drs_params_.gain_heading_rate_p, drs_params_.gain_heading_rate_i, drs_params_.gain_heading_rate_d, drs_params_.saturation_heading_rate, drs_params_.antiwindup_heading_rate);

  // | ------------------ finish loading params ----------------- |

  if (!private_handlers->param_loader->loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  // | ---------------- initialize PID controller --------------- |
  
  pid_state_ = PidController::states::idle;

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool MrsUsvController::activate(const ControlOutput &last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;
  is_active_       = true;

  RCLCPP_INFO(node_->get_logger(), "activated");

  return true;
}

//}

/* //{ deactivate() */

void MrsUsvController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  pid_controller_position_.reset();
  pid_controller_heading_rate_.reset();

  RCLCPP_INFO(node_->get_logger(), "deactivated");
}

//}

/* //{ destroy() */

void MrsUsvController::destroy(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "destroyed");
}

//}

/* updateInactive() //{ */

void MrsUsvController::updateInactive(const mrs_msgs::msg::UavState &uav_state, [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

MrsUsvController::ControlOutput MrsUsvController::updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command) {

  auto drs_params = mrs_lib::get_mutexed(mutex_dynparam_mgr_, drs_params_);
  /* auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_); */

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  // clear all the optional parts of the result
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};


  if (!is_active_) {
    return last_control_output_;
  }

  // | ---------- calculate dt from the last iteration ---------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = rclcpp::Time(uav_state.header.stamp).seconds() - last_update_time_.seconds();
  }

  last_update_time_ = rclcpp::Time(uav_state.header.stamp);

  if (fabs(dt) < 0.001) {
    RCLCPP_WARN(node_->get_logger(), "the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }
  
  if (dt < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "the last odometry message came from future (%.2f s)!", dt);
    dt = 0.01;
  }

  // | -------- check for the available output modalities ------- |

  // you can decide what to return, but it needs to be available
  if (!common_handlers_->control_output_modalities.velocity_hdg_rate) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "desired velocity and heading rate output modality is not available");
    drs_params.publish_zero_outputs = true; 
  }

  // | ---------- extract the detailed model parameters --------- |

  /* if (common_handlers_->detailed_model_params) { */

  /*   mrs_uav_managers::control_manager::DetailedModelParams_t detailed_model_params = common_handlers_->detailed_model_params.value(); */

  /*   ROS_INFO_STREAM_THROTTLE(1.0, "[MrsUsvController]: UAV inertia is: " << detailed_model_params.inertia); */
  /* } */

  // | ------------------ prepare the usv state ----------------- |
  
  double usv_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  Eigen::Vector2d usv_position_vec(uav_state.pose.position.x, uav_state.pose.position.y);

  // | -------------- prepare the control reference ------------- |

  geometry_msgs::msg::PoseStamped position_reference;

  position_reference.header           = tracker_command.header;
  position_reference.pose.position    = tracker_command.position;
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);
  
  Eigen::Vector2d position_reference_vec(position_reference.pose.position.x, position_reference.pose.position.y);

  geometry_msgs::msg::PoseStamped final_position_reference;
  final_position_reference.header           = tracker_command.full_state_prediction.header;
  final_position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.full_state_prediction.heading.back());
  final_position_reference.pose.position    = tracker_command.full_state_prediction.position.back();
  
  Eigen::Vector2d final_position_reference_vec(final_position_reference.pose.position.x, final_position_reference.pose.position.y);
 
  double desired_heading = 0;
  if(drs_params.desired_heading_to_final_reference) {
    desired_heading = getHeading(uav_state.pose.position.x, uav_state.pose.position.y, final_position_reference.pose.position.x, final_position_reference.pose.position.y, last_heading_desired_);
  } else {
    desired_heading = getHeading(uav_state.pose.position.x, uav_state.pose.position.y, position_reference.pose.position.x, position_reference.pose.position.y, last_heading_desired_);
  }
  last_heading_desired_ = desired_heading;  

  // | ----------------- compute control errors ----------------- |
  
  Eigen::Vector2d position_error_vec = position_reference_vec - usv_position_vec;
  Eigen::Vector2d final_position_error_vec = final_position_reference_vec - usv_position_vec;
  double heading_error = mrs_lib::geometry::sradians::diff(desired_heading, usv_heading);

  // | --------------- update the PIDs parameters --------------- |
  pid_controller_position_.setParams(drs_params.gain_position_p, drs_params.gain_position_i, drs_params.gain_position_d, drs_params.saturation_position, drs_params.antiwindup_position);
  pid_controller_heading_rate_.setParams(drs_params.gain_heading_rate_p, drs_params.gain_heading_rate_i, drs_params.gain_heading_rate_d, drs_params.saturation_heading_rate, drs_params.antiwindup_heading_rate);

  // | -------------- compute heading rate command -------------- |
  
  double cmd_heading_rate = 0;
  {
    std::scoped_lock lock(mutex_heading_rate_pid_);
    cmd_heading_rate = pid_controller_heading_rate_.update(heading_error, dt);
  }

  // | ---------------- compute velocity command ---------------- | 
  
  /* auto cmd_velocity = tracker_command.velocity; */
  double cmd_velocity = 0;
  {
    std::scoped_lock lock(mutex_position_pid_);
    cmd_velocity = pid_controller_position_.update(position_error_vec.norm(), dt);
  }

  // | -------------------- apply constraints ------------------- |
  
  if(cmd_heading_rate > drs_params.max_cmd_heading_rate){
    cmd_heading_rate = drs_params.max_cmd_heading_rate; 
  }
  else if(cmd_heading_rate < -drs_params.max_cmd_heading_rate){
    cmd_heading_rate = -drs_params.max_cmd_heading_rate; 
  }
  
  if(cmd_velocity > drs_params.max_cmd_velocity){
    cmd_velocity = drs_params.max_cmd_velocity; 
  }
  else if(cmd_velocity < -drs_params.max_cmd_velocity){
    cmd_velocity = -drs_params.max_cmd_velocity; 
  }

  // | ---------------- prepare the control output --------------- |
 
  double position_error_max = std::max({position_error_vec.norm(), final_position_error_vec.norm()});
  pid_state_ = stateMachineFunction(pid_state_, heading_error, position_error_max, drs_params);

  mrs_msgs::msg::HwApiVelocityHdgRateCmd velocity_heading_rate_cmd;
  if(pid_state_ == PidController::states::idle){
    velocity_heading_rate_cmd.heading_rate = 0; 
    velocity_heading_rate_cmd.velocity.x = 0;
    velocity_heading_rate_cmd.velocity.y = 0;
    velocity_heading_rate_cmd.velocity.z = 0;
  }
  else if(pid_state_ == PidController::states::only_heading){ 
    velocity_heading_rate_cmd.heading_rate = cmd_heading_rate;
    velocity_heading_rate_cmd.velocity.x = 0;
    velocity_heading_rate_cmd.velocity.y = 0;
    velocity_heading_rate_cmd.velocity.z = 0;
  }
  else if(pid_state_ == PidController::states::heading_and_velocity){
    velocity_heading_rate_cmd.heading_rate = cmd_heading_rate; 
    velocity_heading_rate_cmd.velocity.x = cmd_velocity;
    velocity_heading_rate_cmd.velocity.y = 0;
    velocity_heading_rate_cmd.velocity.z = 0;
  }
  else{
    drs_params.publish_zero_outputs = true; 
    RCLCPP_ERROR(node_->get_logger(), "Unknown state of heading rate PID: %d", pid_state_);
  }
  
  // | ----------------- check for zero outputs ----------------- |
  
  if(drs_params.publish_zero_outputs){
    velocity_heading_rate_cmd.velocity.x = 0;
    velocity_heading_rate_cmd.velocity.y = 0;
    velocity_heading_rate_cmd.velocity.z = 0;
    velocity_heading_rate_cmd.heading_rate = 0; 
  }

  // | ----------------- set the control output ----------------- |

  last_control_output_.control_output = velocity_heading_rate_cmd;

  // | --------------- fill in the optional parts --------------- |

  //// it is recommended to fill the optinal parts if you know them

  /// this is used for:
  // * plotting the orientation in the control_refence topic (optional)
  // * checking for attitude control error
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(desired_heading);
  /// IMPORANT
  // The acceleration and heading rate in 3D (expressed in the "fcu" frame of reference) that the UAV will actually undergo due to the control action.
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(0, 0, 0);
  last_control_output_.desired_heading_rate          = cmd_heading_rate;

  last_control_output_.diagnostics.controller_enforcing_constraints = drs_params.enforce_constraints;
  last_control_output_.diagnostics.horizontal_speed_constraint = drs_params.horizontal_speed_constraint;
  last_control_output_.diagnostics.vertical_asc_speed_constraint = drs_params.vertical_asc_speed_constraint;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "MrsUsvController";

  // | ---------------------- debug prints ---------------------- |
  
  if(drs_params.debug) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Heading rate PID state: %s. Heading rate command: %.3f. Velocity command: [%.3f, %.3f, %.3f]", getHeadingRatePidStateName(pid_state_).c_str(), cmd_heading_rate, velocity_heading_rate_cmd.velocity.x, velocity_heading_rate_cmd.velocity.y, velocity_heading_rate_cmd.velocity.z);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Position reference: [%.3f, %.3f, %.3f]. Heading desired: %.3f.", position_reference.pose.position.x, position_reference.pose.position.y,position_reference.pose.position.z, desired_heading);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Current position: [%.3f, %.3f, %.3f]. Current heading: %.3f.", uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z, usv_heading);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Heading error: %.3f. Position error: %.3f.", heading_error, position_error_vec.norm());
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "-------------------------------------------------------------");
  }

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::msg::ControllerStatus MrsUsvController::getStatus() {

  mrs_msgs::msg::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MrsUsvController::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void MrsUsvController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
MrsUsvController::setConstraints([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) {

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

/* stateMachineFunction() //{ */

int  MrsUsvController::stateMachineFunction(int pid_state, double heading_error, double position_error, DrsParams_t drs_params) {
    heading_error = std::abs(heading_error);  
    /* idle state */ 
    if(pid_state == PidController::states::idle){
      if(position_error > drs_params.max_position_error){   
        if(heading_error < drs_params.low_heading_error){
            return PidController::states::heading_and_velocity; 
        }
        return PidController::states::only_heading; 
      }
      // goal reached - reset controllers
      pid_controller_position_.reset();
      pid_controller_heading_rate_.reset();
      return PidController::states::idle;
    } 
    /* only heading state */ 
    else if(pid_state == PidController::states::only_heading){
      /* desired position reached */ 
      if(position_error < drs_params.low_position_error){
        return PidController::states::idle;
      }
      /* still on way */
      if(heading_error < drs_params.low_heading_error){
        return PidController::states::heading_and_velocity; 
      }
      return PidController::states::only_heading; 
    } 
    /* heading and velocity state */ 
    else if(pid_state == PidController::states::heading_and_velocity){ 
      /* desired position reached */ 
      if(position_error < drs_params.low_position_error){
        return PidController::states::idle;
      }
      /* still on way */
      if(heading_error > drs_params.max_heading_error){
        return PidController::states::only_heading; 
      }
      return PidController::states::heading_and_velocity; 
    } 
    RCLCPP_ERROR(node_->get_logger(), "Unknown state in pid_state: %d! Switch to idle.", pid_state);
    return PidController::states::idle;
}

//}

// --------------------------------------------------------------
// |                            utils                           |
// --------------------------------------------------------------

/* getHeading() //{ */
double MrsUsvController::getHeading(double x1, double y1, double x2, double y2, double rescue_heading) {
  if (x1 == x2 && y1 == y2)
  {
    RCLCPP_WARN(node_->get_logger(), "Trying to get heading between one same point!");
    return rescue_heading;
  }

  double heading = atan2(y2 - y1, x2 - x1);

  return heading;
}
//}

/* getHeadingRatePidStateName() //{ */
std::string MrsUsvController::getHeadingRatePidStateName(int heading_rate_pid_state) {
  if (heading_rate_pid_state == PidController::states::only_heading)
  {
    return "only_heading";
  }
  else if (heading_rate_pid_state == PidController::states::heading_and_velocity)
  {
    return "heading_and_velocity";
  }
  else if (heading_rate_pid_state == PidController::states::idle)
  {
    return "idle";
  }
  return "unknown";
}
//}


}  // namespace mrs_usv_controller

}  // namespace mrs_usv_controller_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_usv_controller_plugin::mrs_usv_controller::MrsUsvController, mrs_uav_managers::Controller)
