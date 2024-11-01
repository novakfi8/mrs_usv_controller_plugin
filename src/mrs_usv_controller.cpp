/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <pid.hpp>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_usv_controller_plugin/mrs_usv_controllerConfig.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>

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

  using sradians = mrs_lib::geometry::sradians;

/* //{ class MrsUsvController */

class MrsUsvController : public mrs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  ros::NodeHandle nh_;

  std::string node_name = "MrsUsvController";

  bool is_initialized_ = false;
  bool is_active_      = false;

  double last_heading_desired_ = 0;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                      mutex_drs_;
  typedef mrs_usv_controller_plugin::mrs_usv_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>            Drs_t;
  boost::shared_ptr<Drs_t>                                    drs_;
  void                                                        callbackDrs(mrs_usv_controller_plugin::mrs_usv_controllerConfig& config, uint32_t level);
  DrsConfig_t                                                 drs_params_;
  std::mutex                                                  mutex_drs_params_;


  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  // | ----------------------- controller ----------------------- |
  PIDController pid_controller_heading_rate_;
  std::mutex mutex_heading_rate_pid_;
  
  PIDController pid_controller_position_;
  std::mutex mutex_position_pid_;
  
  int pid_state_;
  std::mutex mutex_pid_state_;
  
  // | -------------------------- utils ------------------------- |
  int  stateMachineFunction(int pid_state, double heading_error, double position_error, DrsConfig_t drs_params);
  double getHeading(double x1, double y1, double x2, double y2, double rescue_heading);
  std::string getHeadingRatePidStateName(int heading_rate_pid_state);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool MrsUsvController::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = ros::Time(0);

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  bool success = true;

  // FYI
  // This method will load the file using `rosparam get`
  //   Pros: you can the full power of the official param loading
  //   Cons: it is slower
  //
  // Alternatives:
  //   You can load the file directly into the ParamLoader as shown below.

  success *= private_handlers->loadConfigFile(ros::package::getPath("mrs_usv_controller_plugin") + "/config/mrs_usv_controller.yaml");

  if (!success) {
    ROS_ERROR("[MrsUsvController]: could not load all parameters!");
    return false;
  }

  mrs_lib::ParamLoader param_loader(nh_, "MrsUsvController");

  // This is the alternaive way of loading the config file.
  //
  // Files loaded using this method are prioritized over ROS params.
  //
  // param_loader.addYamlFile(ros::package::getPath("mrs_usv_tracker_plugin") + "/config/mrs_usv_tracker.yaml");

  param_loader.loadParam("debug", drs_params_.debug);
  param_loader.loadParam("publish_zero_outputs", drs_params_.publish_zero_outputs);
  
  param_loader.loadParam("desired_heading_to_final_reference", drs_params_.desired_heading_to_final_reference);
  param_loader.loadParam("max_cmd_heading_rate", drs_params_.max_cmd_heading_rate);
  param_loader.loadParam("max_cmd_velocity", drs_params_.max_cmd_velocity);
  param_loader.loadParam("enforce_constraints", drs_params_.enforce_constraints);
  param_loader.loadParam("horizontal_speed_constraint", drs_params_.horizontal_speed_constraint);
  param_loader.loadParam("vertical_asc_speed_constraint", drs_params_.vertical_asc_speed_constraint);
  
  param_loader.loadParam("low_position_error", drs_params_.low_position_error);
  param_loader.loadParam("max_position_error", drs_params_.max_position_error);
  param_loader.loadParam("low_heading_error", drs_params_.low_heading_error);
  param_loader.loadParam("max_heading_error", drs_params_.max_heading_error);
 
  param_loader.loadParam("kp_position", drs_params_.kp_position);
  param_loader.loadParam("kd_position", drs_params_.kd_position);
  param_loader.loadParam("ki_position", drs_params_.ki_position);
  param_loader.loadParam("saturation_position", drs_params_.saturation_position);
  param_loader.loadParam("antiwindup_position", drs_params_.antiwindup_position);
  pid_controller_position_.setParams(drs_params_.kp_position, drs_params_.kd_position, drs_params_.ki_position, drs_params_.saturation_position, drs_params_.antiwindup_position);
  
  param_loader.loadParam("kp_heading_rate", drs_params_.kp_heading_rate);
  param_loader.loadParam("kd_heading_rate", drs_params_.kd_heading_rate);
  param_loader.loadParam("ki_heading_rate", drs_params_.ki_heading_rate);
  param_loader.loadParam("saturation_heading_rate", drs_params_.saturation_heading_rate);
  param_loader.loadParam("antiwindup_heading_rate", drs_params_.antiwindup_heading_rate);
  pid_controller_heading_rate_.setParams(drs_params_.kp_heading_rate, drs_params_.kd_heading_rate, drs_params_.ki_heading_rate, drs_params_.saturation_heading_rate, drs_params_.antiwindup_heading_rate);

  // | ------------------ finish loading params ----------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUsvController]: could not load all parameters!");
    return false;
  }

  // | ---------------- initialize PID controller --------------- |
  
  pid_state_ = PidController::states::idle;

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&MrsUsvController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUsvController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool MrsUsvController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;

  is_active_ = true;

  ROS_INFO("[MrsUsvController]: activated");

  return true;
}

//}

/* //{ deactivate() */

void MrsUsvController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  pid_controller_position_.reset();
  pid_controller_heading_rate_.reset();

  ROS_INFO("[MrsUsvController]: deactivated");
}

//}

/* updateInactive() //{ */

void MrsUsvController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

MrsUsvController::ControlOutput MrsUsvController::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

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
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {
    ROS_DEBUG("[MrsUsvController]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }
  
  if (dt < 0.0) {
    ROS_ERROR("[MrsUsvController]: the last odometry message came from future (%.2f s)!", dt);
    dt = 0.01;
  }

  // | -------- check for the available output modalities ------- |

  // you can decide what to return, but it needs to be available
  if (!common_handlers_->control_output_modalities.velocity_hdg_rate) {
    ROS_ERROR_THROTTLE(1.0, "[MrsUsvController]: desired velocity and heading rate output modality is not available");
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

  geometry_msgs::PoseStamped position_reference;

  position_reference.header           = tracker_command.header;
  position_reference.pose.position    = tracker_command.position;
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);
  
  Eigen::Vector2d position_reference_vec(position_reference.pose.position.x, position_reference.pose.position.y);

  geometry_msgs::PoseStamped final_position_reference;
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
  double heading_error = sradians::diff(desired_heading, usv_heading);

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

  mrs_msgs::HwApiVelocityHdgRateCmd velocity_heading_rate_cmd;
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
    ROS_ERROR("[MrsUsvController]: Unknown state of heading rate PID: %d", pid_state_);
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
    ROS_INFO_THROTTLE(1.0, "[MrsUsvController]: Heading rate PID state: %s. Heading rate command: %.3f. Velocity command: [%.3f, %.3f, %.3f]", getHeadingRatePidStateName(pid_state_).c_str(), cmd_heading_rate, velocity_heading_rate_cmd.velocity.x, velocity_heading_rate_cmd.velocity.y, velocity_heading_rate_cmd.velocity.z);
    ROS_INFO_THROTTLE(1.0, "[MrsUsvController]: Position reference: [%.3f, %.3f, %.3f]. Heading desired: %.3f.", position_reference.pose.position.x, position_reference.pose.position.y,position_reference.pose.position.z, desired_heading);
    ROS_INFO_THROTTLE(1.0, "[MrsUsvController]: Current position: [%.3f, %.3f, %.3f]. Current heading: %.3f.", uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z, usv_heading);
    ROS_INFO_THROTTLE(1.0, "[MrsUsvController]: Heading error: %.3f. Position error: %.3f.", heading_error, position_error_vec.norm());
    ROS_INFO_THROTTLE(1.0, "[MrsUsvController]: -------------------------------------------------------------");
  }

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus MrsUsvController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MrsUsvController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState& new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void MrsUsvController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr MrsUsvController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  ROS_INFO("[MrsUsvController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

/* stateMachineFunction() //{ */

int  MrsUsvController::stateMachineFunction(int pid_state, double heading_error, double position_error, DrsConfig_t drs_params) {
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
    ROS_ERROR("[MrsUsvController]: Unknown state in pid_state: %d! Switch to idle.", pid_state);
    return PidController::states::idle;
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void MrsUsvController::callbackDrs(mrs_usv_controller_plugin::mrs_usv_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);
  
  {
    std::scoped_lock lock(mutex_position_pid_, mutex_heading_rate_pid_);
    pid_controller_position_.setParams(drs_params_.kp_position, drs_params_.kd_position, drs_params_.ki_position, drs_params_.saturation_position, drs_params_.antiwindup_position); 
    pid_controller_heading_rate_.setParams(drs_params_.kp_heading_rate, drs_params_.kd_heading_rate, drs_params_.ki_heading_rate, drs_params_.saturation_heading_rate, drs_params_.antiwindup_heading_rate); 
  } 

  ROS_INFO("[MrsUsvController]: dynamic reconfigure params updated");
}

//}

// --------------------------------------------------------------
// |                            utils                           |
// --------------------------------------------------------------

/* getHeading() //{ */
double MrsUsvController::getHeading(double x1, double y1, double x2, double y2, double rescue_heading) {
  if (x1 == x2 && y1 == y2)
  {
    ROS_WARN("[%s] Trying to get heading between one same point!", node_name.c_str()); 
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_usv_controller_plugin::mrs_usv_controller::MrsUsvController, mrs_uav_managers::Controller)
