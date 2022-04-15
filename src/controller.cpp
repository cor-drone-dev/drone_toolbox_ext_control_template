#include "drone_toolbox_ext_control_template/controller.h"

/************************************ INIT FUNCTIONS ************************************/
bool Controller::initialize()
{
    // Initialize ROS interface
    if (!initRosInterface()) {
        CONTROLLER_ERROR("Failed to initialize ROS interface");
        return false;
    }

    // Initialize controller settings
    goal_pos_ = {1, 1, 2};
    first_state_received_ = false;
    control_method_ = POS_CTRL;

    // Construct default position message
    // See:
    // - https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/PositionTarget.msg
    // - https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    // Processed via:
    // - https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/setpoint_raw.cpp#L164
    // - https://github.com/PX4/PX4-Autopilot/blob/77a37c26bf7696d6ce8658a94339e3e28de340c4/src/modules/mavlink/mavlink_receiver.cpp#L1004
    pos_msg_.header.frame_id = "base_link";
    pos_msg_.coordinate_frame = 1;
    pos_msg_.type_mask = 4088; //only look at position setpoints (binary: 0000 1111 1111 1000)
    pos_msg_.velocity.x = NAN;
    pos_msg_.velocity.y = NAN;
    pos_msg_.velocity.z = NAN;
    pos_msg_.acceleration_or_force.x = NAN;
    pos_msg_.acceleration_or_force.y = NAN;
    pos_msg_.acceleration_or_force.z = NAN;
    pos_msg_.yaw = NAN;
    pos_msg_.yaw_rate = NAN;

    // Construct default velocity message
    vel_msg_.header.frame_id = "base_link";
    vel_msg_.coordinate_frame = 1;
    vel_msg_.type_mask = 4039; //only look at velocity setpoints (binary: 0000 1111 1100 0111)
    vel_msg_.position.x = NAN;
    vel_msg_.position.y = NAN;
    vel_msg_.position.z = NAN;
    vel_msg_.acceleration_or_force.x = NAN;
    vel_msg_.acceleration_or_force.y = NAN;
    vel_msg_.acceleration_or_force.z = NAN;
    vel_msg_.yaw = NAN;
    vel_msg_.yaw_rate = NAN;

    // Define control loop timer at specified frequency: should be higher than 2Hz
    loop_timer_ = nh_.createTimer(ros::Duration(1/loop_frequency_), &Controller::loop, this);
    loop_timer_.stop();
    timer_running_ = false;

    return true;
}

bool Controller::initRosInterface()
{
    // Enable debug printing at least for the first part of the algorithm
    enable_debug_ = true;

    // Store ROS parameters
    if (!getRosParameters()) {
        CONTROLLER_ERROR("Failed to obtain ROS parameters!");
        return false;
    }

    // Safety check: remove in case it is desired to test the controller in the lab
    if (!is_sim_) {
        CONTROLLER_ERROR("Sim is set to false, but this is not allowed in this template controller!");
        exit(1);
    }

    // ROS subscribers and publishers
    state_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &Controller::stateCallback, this);
    pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // ROS service servers and clients
    enable_control_server_ = nh_.advertiseService("/px4_ext_cont_enable", &Controller::enableControlCallback, this);
    disable_control_server_ = nh_.advertiseService("/px4_ext_cont_disable", &Controller::disableControlCallback, this);
    mission_finished_client_ = nh_.serviceClient<std_srvs::Trigger>("/px4_mission_finished_ext_cont");

    return true;
}

bool Controller::getRosParameters()
{
    // Required
    if (!retrieveParameter(nh_, "goal_pos", goal_pos_)) return false;
    if (!retrieveParameter(nh_, "enable_debug", enable_debug_)) return false;
    // Non-required with default value
    retrieveParameter(nh_, "is_sim", is_sim_, is_sim_default_);
    retrieveParameter(nh_, "loop_frequency", loop_frequency_, loop_frequency_default_);
    retrieveParameter(nh_, "dist_thres", dist_thres_, dist_thres_default_);

    return true;
}
/****************************************************************************************/


/************************************ LOOP FUNCTIONS ************************************/
void Controller::loop(const ros::TimerEvent &event)
{
    // Check for reaching goal
    missionFinishedCheck();
    
    // Calculate and publish next control command
    controllerExecution();
}

void Controller::missionFinishedCheck()
{
    if (getEuclideanDistance3d(cur_pos_, goal_pos_) < dist_thres_ && timer_running_) {
        if (mission_finished_client_.call(mission_finished_srv_)) {
            CONTROLLER_INFO("Mission finished! Transferred back control to PX4 control interface and got the following message back: '" << mission_finished_srv_.response.message << "'");
            loop_timer_.stop();
            timer_running_ = false;
        } else {
            CONTROLLER_WARN_ONCE("Mission finished, but failed to transfer back control to PX4 control interface! Will try again every control loop execution until success");
        }
    }
}

void Controller::controllerExecution()
{
    // Insert controller code here (possibly create different classes for different controller parts to keep the code modular. For example, use multiple instances of a PID class to control thrust and x/y positions separately.

    // The code below is provided as an example to show the different ways of interfacing with MAVROS and PX4
    switch (control_method_) {
        case POS_CTRL:
            pos_msg_.header.stamp = ros::Time::now();
            pos_msg_.position.x = 1;
            pos_msg_.position.y = 1;
            pos_msg_.position.z = 2;

            pos_pub_.publish(pos_msg_);
            break;

        case VEL_CTRL:
            vel_msg_.header.stamp = ros::Time::now();
            vel_msg_.velocity.x = 0.3;
            vel_msg_.velocity.y = 0.7;
            vel_msg_.velocity.z = 0.2;

            vel_pub_.publish(vel_msg_);
            break;
    }
}
/****************************************************************************************/


/********************************** CALLBACK FUNCTIONS **********************************/
void Controller::reconfigureCallback(drone_toolbox_ext_control_template::ControllerConfig& config, uint32_t level)
{
    if (first_reconfig_cb_) {
        first_reconfig_cb_ = false;
        config.enable_debug = enable_debug_;
    } else {
        enable_debug_ = config.enable_debug;
    }

    switch (config.control_select) {
        case 0:
            control_method_ = POS_CTRL;
            break;

        case 1:
            control_method_ = VEL_CTRL;
            break;

        default:
            CONTROLLER_WARN("Invalid control method selected. Switching to position control.");
            control_method_ = POS_CTRL;
            break;
    }
}

bool Controller::enableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (!first_state_received_) {
        CONTROLLER_WARN("enableControlCallback called: control loop can only be started if first state is received!");
        res.success = false;
        res.message = "Control loop can only be started if first state is received!";
        return false;
    } else {
        CONTROLLER_INFO("enableControlCallback called: starting control loop");
        loop_timer_.start();
        timer_running_ = true;
        res.success = true;
        res.message = "Enabled control loop";
        return true;
    }
}

bool Controller::disableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    CONTROLLER_INFO("disableControlCallback called: stopping control loop");
    loop_timer_.stop();
    timer_running_ = false;
    res.success = true;
    res.message = "Disabled control loop";
    return true;
}

void Controller::stateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!first_state_received_) {
        first_state_received_ = true;
    }
    
    cur_odom_ = *msg;
    cur_pose_ = cur_odom_.pose.pose;
    cur_pos_ = {cur_pose_.position.x, cur_pose_.position.y, cur_pose_.position.z};
}
/****************************************************************************************/
