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
    mission_finished_ = false;
    first_state_received_ = false;
    control_method_ = POS_CTRL;

    // Set timeout for error printing
    print_timeout_ = ros::Duration(1.0);

    // Construct default position target message (used for position, velocity and yaw/yaw rate commands)
    pos_target_msg_.header.frame_id = "base_link";
    pos_target_msg_.coordinate_frame = 1; //binary: 0000 1101 1111 1111 => ignore everything
    pos_target_msg_.type_mask = 3583;
    pos_target_msg_.position.x = NAN;
    pos_target_msg_.position.y = NAN;
    pos_target_msg_.position.z = NAN;
    pos_target_msg_.velocity.x = NAN;
    pos_target_msg_.velocity.y = NAN;
    pos_target_msg_.velocity.z = NAN;
    pos_target_msg_.acceleration_or_force.x = NAN;
    pos_target_msg_.acceleration_or_force.y = NAN;
    pos_target_msg_.acceleration_or_force.z = NAN;
    pos_target_msg_.yaw = NAN;
    pos_target_msg_.yaw_rate = NAN;

    // Construct default attitude target message (used for attitude and thrust commands)
    att_target_msg_.header.frame_id = "base_link";
    att_target_msg_.type_mask = 255; //binary: 1111 1111 => ignore everything
    att_target_msg_.orientation.x = NAN;
    att_target_msg_.orientation.y = NAN;
    att_target_msg_.orientation.z = NAN;
    att_target_msg_.orientation.w = NAN;
    att_target_msg_.body_rate.x = NAN;
    att_target_msg_.body_rate.y = NAN;
    att_target_msg_.body_rate.z = NAN;
    att_target_msg_.thrust = NAN;

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
    state_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &Controller::stateCallback, this);
    pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pos_yaw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pos_yawrate_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    rpyrt_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", 1);

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
        mission_finished_ = true;
    }

    if (mission_finished_) {
        if (mission_finished_client_.call(mission_finished_srv_)) {
            CONTROLLER_INFO("Mission finished! Transferred back control to PX4 control interface and got the following message back: '" << mission_finished_srv_.response.message << "'");
            loop_timer_.stop();
            timer_running_ = false;
        } else {
            if (ros::Time::now() - last_time_ > print_timeout_) {
                CONTROLLER_ERROR("Mission finished, but failed to transfer back control to PX4 control interface! Will try again every control loop execution until success");
                last_time_ = ros::Time::now();
            }
        }
    }
}

void Controller::controllerExecution()
{
    /******************************************************************************************************* /
        Insert custom controller code here.
        Possibly create different classes for different controller parts to keep the code modular.
        For example, use multiple instances of a PID class to control thrust and x/y positions separately.
    /********************************************************************************************************/

    /* The code below is provided as an example to show the different ways of interfacing with MAVROS and PX4 */

    // A standard sine and cosine wave for velocity and attitude commands
    ros::Time cur_time = ros::Time::now();
    double f = 0.25;
    double t = ros::Duration(cur_time - loop_start_time_).toSec();
    double sin_val = sin(2*M_PI*f*t);
    double cos_val = cos(2*M_PI*f*t);

    // Calculate and publish control commands
    switch (control_method_) {
        case POS_CTRL:
            pos_target_msg_.header.stamp = ros::Time::now();
            pos_target_msg_.type_mask = 3576; //binary: 0000 1101 1111 1000 => ignore everything except position setpoints
            pos_target_msg_.position.x = 1;
            pos_target_msg_.position.y = 1;
            pos_target_msg_.position.z = 2;

            pos_pub_.publish(pos_target_msg_);
            break;

        case POS_YAW_CTRL:
            // TODO Edit PX4 controller code to allow this interface to work properly
            pos_target_msg_.header.stamp = ros::Time::now();
            pos_target_msg_.type_mask = 2552; //binary: 0000 1001 1111 1000 => ignore everything except position and yaw setpoints
            pos_target_msg_.position.x = 1;
            pos_target_msg_.position.y = 1;
            pos_target_msg_.position.z = 2;
            pos_target_msg_.yaw = M_PI;

            pos_yaw_pub_.publish(pos_target_msg_);
            break;

        case POS_YAWRATE_CTRL:
            pos_target_msg_.header.stamp = ros::Time::now();
            pos_target_msg_.type_mask = 1528; //binary: 0000 0101 1111 1000 => ignore everything except position and yaw rate setpoints
            pos_target_msg_.position.x = 1;
            pos_target_msg_.position.y = 1;
            pos_target_msg_.position.z = 2;
            pos_target_msg_.yaw_rate = 1;

            pos_yawrate_pub_.publish(pos_target_msg_);
            break;

        case VEL_CTRL:
            pos_target_msg_.header.stamp = ros::Time::now();
            pos_target_msg_.type_mask = 3527; //binary: 0000 1101 1100 0111 => ignore everything except velocity setpoints
            pos_target_msg_.velocity.x = 0.5*cos_val;
            pos_target_msg_.velocity.y = 0.5*sin_val;
            pos_target_msg_.velocity.z = 0;

            vel_pub_.publish(pos_target_msg_);
            break;

        case ATT_CTRL:
        {
            // Define orientation in ZYX Euler angles (rad) and convert to quaternion
            tf2Scalar roll = M_PI/10*sin_val, pitch = M_PI/10*cos_val, yaw = 0;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw); //fixed angles RPY is equivalent to ZYX Euler angles

            // Define thrust and convert to input desired by PX4: [0.1, 0.9] (assuming linear relation)
            // Used for attitude commands
            double desired_thrust = 9.81;
            double thrust = (desired_thrust - 9.81) / 22.629 + 0.674;

            // Fill and send message
            att_target_msg_.header.stamp = ros::Time::now();
            att_target_msg_.type_mask = 63; //binary: 0011 1111 => ignore everything except attitude and thrust setpoints
            att_target_msg_.orientation.x = q.x();
            att_target_msg_.orientation.y = q.y();
            att_target_msg_.orientation.z = q.z();
            att_target_msg_.orientation.w = q.w();
            att_target_msg_.thrust = thrust;

            att_pub_.publish(att_target_msg_);
            break;
        }

        case RPYRT_CTRL:
        {
            // Define attitude (rad) and yaw rate (rad/s) commands
            double roll = M_PI/10*sin_val;
            double pitch = M_PI/10*cos_val;
            double yaw_rate = 1;

            // Define thrust and convert to input desired by PX4: [0.1, 0.9] (assuming linear relation)
            // Used for attitude commands
            double desired_thrust = 9.81;
            double thrust = (desired_thrust - 9.81) / 22.629 + 0.674;

            // Fill and send message
            rpyrt_msg_.header.stamp = ros::Time::now();
            rpyrt_msg_.roll = roll;
            rpyrt_msg_.pitch = pitch;
            rpyrt_msg_.yaw_rate = yaw_rate;
            rpyrt_msg_.thrust.x = NAN;
            rpyrt_msg_.thrust.y = NAN;
            rpyrt_msg_.thrust.z = thrust;

            rpyrt_pub_.publish(rpyrt_msg_);
            break;
        }
    }
}
/****************************************************************************************/


/********************************** CALLBACK FUNCTIONS **********************************/
void Controller::reconfigureCallback(drone_toolbox_ext_control_template::ControllerConfig& config, uint32_t level)
{
    if (level & 1) {
        if (first_reconfig_cb_) {
            first_reconfig_cb_ = false;
            config.enable_debug = enable_debug_;
        } else {
            enable_debug_ = config.enable_debug;
        }
    }

    if (level & 2) {
        switch (config.control_select) {
            case 0:
                CONTROLLER_INFO("Switching to position control");
                control_method_ = POS_CTRL;
                break;

            case 1:
                CONTROLLER_INFO("Combined position and yaw control currently not available! Switching to position control");
                control_method_ = POS_CTRL;
                break;

            case 2:
                CONTROLLER_INFO("Switching to combined position and yaw rate control");
                control_method_ = POS_YAWRATE_CTRL;
                break;

            case 3:
                CONTROLLER_INFO("Switching to velocity control");
                control_method_ = VEL_CTRL;
                break;

            case 4:
                CONTROLLER_INFO("Switching to attitude control");
                control_method_ = ATT_CTRL;
                break;

            case 5:
                CONTROLLER_INFO("Switching to roll, pitch, yaw rate and thrust control");
                control_method_ = RPYRT_CTRL;
                break;

            default:
                CONTROLLER_WARN("Invalid control method selected! Switching to position control");
                control_method_ = POS_CTRL;
                break;
        }
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
        mission_finished_ = false;
        loop_timer_.start();
        loop_start_time_ = ros::Time::now();
        last_time_ = loop_start_time_;
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
