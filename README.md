# Drone toolbox external controller template
This repository serves as a controller template for interfacing with the PX4 control interface defined in the [drone_toolbox](https://github.com/cor-drone-dev/drone_toolbox) repository. Follow the [corresponding installation instructions](https://github.com/cor-drone-dev/drone_toolbox/blob/master/doc/Installation_instructions.md#clone-template-controller) in [drone_toolbox](https://github.com/cor-drone-dev/drone_toolbox) to get started using this repository.

To interface a controller with the PX4 control interface, it should have the following ROS components:
1. [Service server to start the control loop](#start-control-loop-server)
2. [Service server to stop the control loop](#stop-control-loop-server)
3. [Service client to stop the control loop and indicate that the mission is finished to the PX4 control interface](#mission-finished-client)
4. [State subscriber subscribing to the odometry data of the robot](#state-subscriber)
5. [Control command publisher publishing the control output](#control-command-publisher)

These components are detailed below according to the implementation in this repository.



## Start control loop server
Create the server:
```
enable_control_server_ = nh_.advertiseService("/px4_ext_cont_enable", &Controller::enableControlCallback, this);
```
and define the callback function:
```
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
        loop_start_time_ = ros::Time::now();
        timer_running_ = true;
        res.success = true;
        res.message = "Enabled control loop";
        return true;
    }
}
```



## Stop control loop server
Create the server:
```
disable_control_server_ = nh_.advertiseService("/px4_ext_cont_disable", &Controller::disableControlCallback, this);
```
and define the corresponding callback function:
```
bool Controller::disableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    CONTROLLER_INFO("disableControlCallback called: stopping control loop");
    loop_timer_.stop();
    timer_running_ = false;
    res.success = true;
    res.message = "Disabled control loop";
    return true;
}
```



## Mission finished client
Create the client:
```
mission_finished_client_ = nh_.serviceClient<std_srvs::Trigger>("/px4_mission_finished_ext_cont");
```
and check whether the mission is finished inside the control loop:
```
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
```
*Note: determining whether the mission finished is finished or not is done in this template based on a goal position (loaded as a ROS parameter from *config/controller.yaml*). However, one can implement this using an arbitrary condition that does not necessarily depend on a goal position.*



## State subscriber
Create the subscriber:
```
state_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &Controller::stateCallback, this);
```
and its corresponding callback function:
```
void Controller::stateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!first_state_received_) {
        first_state_received_ = true;
    }
    
    cur_odom_ = *msg;
    cur_pose_ = cur_odom_.pose.pose;
    cur_pos_ = {cur_pose_.position.x, cur_pose_.position.y, cur_pose_.position.z};
}
```



## Control command publisher
Depending on the type of controller, it might output different control commands. This template contains several control command interfaces that can be used as a reference, including:
- [Position](#position-control)
- [Position and yaw](#position-and-yaw-control)
- [Position and yaw rate](#position-and-yaw-rate-control)
- [Velocity](#velocity-control)
- [Attitude](#attitude-control)
- [Roll, pitch, yaw rate and thrust (RPYrT)](#rpyrt-control)

The code for publishing the different control commands is detailed below.

The [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message is used for position and velocity commands (can also be used for acceleration and force commands, but this is not part of the template). After being published on the `/mavros/setpoint_raw/local` topic, this message is converted by MAVROS into a MAVLink message [here](https://github.com/cor-drone-dev/mavros/blob/RPYrT_support/mavros/src/plugins/setpoint_raw.cpp#L167) (see [this link](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) for the MAVLink message definition). Thereafter, it is received by the PX4 MAVLink receiver module [here](https://github.com/cor-drone-dev/PX4-Autopilot/blob/hg_drone_RPYrT_support/src/modules/mavlink/mavlink_receiver.cpp#L871) and subsequently taken as target in the PX4 position and velocity controller stack [here](https://github.com/cor-drone-dev/PX4-Autopilot/blob/hg_drone_RPYrT_support/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L293).

The [mavros_msgs/AttitudeTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/AttitudeTarget.html) message is used for attitude commands (can also be used for body rate commands, but this is not part of the template). After being published on the `/mavros/setpoint_raw/attitude` topic, this message is converted by MAVROS into a MAVLink message [here](https://github.com/cor-drone-dev/mavros/blob/RPYrT_support/mavros/src/plugins/setpoint_raw.cpp#L239) (see [this link](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) for the MAVLink message definition). Thereafter, it is received by the PX4 MAVLink receiver module [here](https://github.com/cor-drone-dev/PX4-Autopilot/blob/hg_drone_RPYrT_support/src/modules/mavlink/mavlink_receiver.cpp#L1397) and subsequently taken as target in the PX4 attitude controller stack [here](https://github.com/cor-drone-dev/PX4-Autopilot/blob/hg_drone_RPYrT_support/src/modules/mc_att_control/mc_att_control_main.cpp#L249).

The [mav_msgs/RollPitchYawrateThrust](https://docs.ros.org/en/indigo/api/mav_msgs/html/msg/RollPitchYawrateThrust.html) message is used for roll, pitch, yaw rate and thrust commands. After being published on the `/mavros/setpoint_raw/roll_pitch_yawrate_thrust` topic, this message is converted by MAVROS into a MAVLink message [here](https://github.com/cor-drone-dev/mavros/blob/RPYrT_support/mavros/src/plugins/setpoint_raw.cpp#L282). The rest of the communication chain is the same as for the attitude target.

*Note: in this template, switching between the different commands is possible using ROS dynamic reconfiguration (see *cfg/controller.cfg* for the definition of all reconfigurable parameters. They are processed in the `reconfigureCallback` function inside *src/controller.cpp*).*

*Note 2: to complete a full experiment (i.e., from taking off to landing), please select a variant of the position control options, since they are able to reach the goal position. The other control options are meant to show how to use these commands rather than moving to a goal location.*


### Position control
Create the publisher:
```
pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
pos_target_msg_.header.stamp = ros::Time::now();
pos_target_msg_.type_mask = 3576; //binary: 0000 1101 1111 1000 => ignore everything except position setpoints
pos_target_msg_.position.x = 1;
pos_target_msg_.position.y = 1;
pos_target_msg_.position.z = 2;
```
and publish:
```
pos_pub_.publish(pos_target_msg_);
```


### Position and yaw control
Create the publisher:
```
pos_yaw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
pos_target_msg_.header.stamp = ros::Time::now();
pos_target_msg_.type_mask = 2552; //binary: 0000 1001 1111 1000 => ignore everything except position and yaw setpoints
pos_target_msg_.position.x = 1;
pos_target_msg_.position.y = 1;
pos_target_msg_.position.z = 2;
pos_target_msg_.yaw = M_PI;
```
and publish:
```
pos_yaw_pub_.publish(pos_target_msg_);
```

*Note: this type of control is currently not supported, since the yaw rate setpoint of the attitude controller gets overwritten by the yaw rate command to allow PX4 to execute RPYrT commands (see below).*


### Position and yaw rate control
Create the publisher:
```
pos_yawrate_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
pos_target_msg_.header.stamp = ros::Time::now();
pos_target_msg_.type_mask = 1528; //binary: 0000 0101 1111 1000 => ignore everything except position and yaw rate setpoints
pos_target_msg_.position.x = 1;
pos_target_msg_.position.y = 1;
pos_target_msg_.position.z = 2;
pos_target_msg_.yaw_rate = 1;
```
and publish:
```
pos_yawrate_pub_.publish(pos_target_msg_);
```


### Velocity control
Create the publisher:
```
vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
pos_target_msg_.header.stamp = ros::Time::now();
pos_target_msg_.type_mask = 3527; //binary: 0000 1101 1100 0111 => ignore everything except velocity setpoints
pos_target_msg_.velocity.x = 0.5*cos_val;
pos_target_msg_.velocity.y = 0.5*sin_val;
pos_target_msg_.velocity.z = 0;
```
and publish:
```
vel_pub_.publish(pos_target_msg_);
```


### Attitude control
Create the publisher:
```
att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
```
fill the [mavros_msgs/AttitudeTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/AttitudeTarget.html) message:
```
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
```
and publish:
```
att_pub_.publish(att_target_msg_);
```
*Note: the thrust value provided to PX4 should be given as a value between 0 and 1 (see [this link](https://docs.px4.io/master/en/concept/mixing.html#multirotor-mixer) and [this code line](https://github.com/cor-drone-dev/mavros/blob/5c38353bea11af5139d81dc71d5d27df77b4df4f/mavros/src/plugins/setpoint_raw.cpp#L259)), because the real thrust the drone experiences depends on the specific hardware present on the platform (ESCs and motors). Therefore, before controlling the drone with a thrust value, first make sure to derive the relation between real thrust and commanded thrust. This relation depends on the battery voltage. This constraint on the commanded thrust holds for all messages containing a thrust value, including the RPYrT control message.*


### RPYrT control
Create the publisher:
```
rpyrt_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", 1);
```
fill the [mav_msgs/RollPitchYawrateThrust](https://docs.ros.org/en/indigo/api/mav_msgs/html/msg/RollPitchYawrateThrust.html) message:
```
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
```
and publish:
```
rpyrt_pub_.publish(rpyrt_msg_);
```
