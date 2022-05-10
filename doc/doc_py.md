# Drone toolbox external controller template (Python)

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
self.enable_control_srv = rospy.Service("/px4_ext_cont_enable", Trigger, self.enable_control_callback)
```
and define the callback function:
```
def enable_control_callback(self, req: Trigger._request_class) -> Trigger._response_class:
        
    res = Trigger._response_class()

    if not self.first_state_received:
        rospy.logwarn('[CONTROLLER]: enableControlCallback called: control loop can only be started if first state is received!"')
        res.success = False
        res.message = "Control loop can only be started if first state is received!"
    else:
        rospy.loginfo('[CONTROLLER]: enableControlCallback called: starting control loop')
        self.mission_finished = False
        self.loop_timer.restart()
        self.loop_start_time = rospy.Time.now()
        self.last_time = self.loop_start_time
        self.timer_running = True
        res.success = True
        res.message = "Enabled control loop"
    
    return res
```



## Stop control loop server
Create the server:
```
self.disable_control_srv = rospy.Service("/px4_ext_cont_disable", Trigger, self.disable_control_callback)
```
and define the corresponding callback function:
```
def disable_control_callback(self, req: Trigger._request_class) -> Trigger._response_class:
        
    res = Trigger._response_class()
    rospy.loginfo('[CONTROLLER]: disableControlCallback called: stopping control loop')
    self.loop_timer.pause()
    self.timer_running = False
    res.success = True
    res.message = "Disabled control loop"
    return res
```



## Mission finished client
Create the client:
```
self.mission_finished_clt = rospy.ServiceProxy("/px4_mission_finished_ext_cont", Trigger)
```
and check whether the mission is finished inside the control loop:
```
def mission_finished_check(self) -> None:
        
    distance = np.linalg.norm(np.asarray(self.goal_pos) - np.asarray(self.cur_pos).transpose())
    if distance < self.dist_thres and self.timer_running:
        self.mission_finished = True
    if self.mission_finished:
        res = self.mission_finished_clt(Trigger._request_class())
        if res:
            rospy.loginfo('[CONTROLLER]: Mission finished! Transferred back control to PX4 control interface and got the following message back: ' + res.message)
            self.loop_timer.pause()
            self.timer_running = False
        else:
            if rospy.Time.now() - self.last_time > self.print_timeout:
                rospy.logerr('[CONTROLLER]: Mission finished, but failed to transfer back control to PX4 control interface! Will try again every control loop execution until success')
                self.last_time = rospy.Time.now()

```
*Note: determining whether the mission finished is finished or not is done in this template based on a goal position (loaded as a ROS parameter from *config/controller.yaml*). However, one can implement this using an arbitrary condition that does not necessarily depend on a goal position.*



## State subscriber
Create the subscriber:
```
self.state_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.state_callback)
```
and its corresponding callback function:
```
def state_callback(self, msg: Odometry) -> None:
        if not self.first_state_received:
            self.first_state_received = True
        self.cur_odom_msg = msg
        self.cur_pose_msg = self.cur_odom_msg.pose.pose
        self.cur_pos[0] = self.cur_pose_msg.position.x
        self.cur_pos[1] = self.cur_pose_msg.position.y
        self.cur_pos[2] = self.cur_pose_msg.position.z
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

The [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message is used for position and velocity commands (can also be used for acceleration and force commands, but this is not part of the template). After being published on the `/mavros/setpoint_raw/local` topic, this message is converted by MAVROS into a MAVLink message [here](https://github.com/dbenders1/mavros/blob/RPYrT_support/mavros/src/plugins/setpoint_raw.cpp#L167) (see [this link](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) for the MAVLink message definition). Thereafter, it is received by the PX4 MAVLink receiver module [here](https://github.com/dbenders1/PX4-Autopilot/blob/hovergames_drone_RPYrT/src/modules/mavlink/mavlink_receiver.cpp#L996) and subsequently taken as target in the PX4 position and velocity controller stack [here](https://github.com/dbenders1/PX4-Autopilot/blob/hovergames_drone_RPYrT/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L320).

The [mavros_msgs/AttitudeTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/AttitudeTarget.html) message is used for attitude commands (can also be used for body rate commands, but this is not part of the template). After being published on the `/mavros/setpoint_raw/attitude` topic, this message is converted by MAVROS into a MAVLink message [here](https://github.com/dbenders1/mavros/blob/RPYrT_support/mavros/src/plugins/setpoint_raw.cpp#L239) (see [this link](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) for the MAVLink message definition). Thereafter, it is received by the PX4 MAVLink receiver module [here](https://github.com/dbenders1/PX4-Autopilot/blob/hovergames_drone_RPYrT/src/modules/mavlink/mavlink_receiver.cpp#L1532) and subsequently taken as target in the PX4 attitude controller stack [here](https://github.com/dbenders1/PX4-Autopilot/blob/hovergames_drone_RPYrT/src/modules/mc_att_control/mc_att_control_main.cpp#L249).

The [mav_msgs/RollPitchYawrateThrust](https://docs.ros.org/en/indigo/api/mav_msgs/html/msg/RollPitchYawrateThrust.html) message is used for roll, pitch, yaw rate and thrust commands. After being published on the `/mavros/setpoint_raw/roll_pitch_yawrate_thrust` topic, this message is converted by MAVROS into a MAVLink message [here](https://github.com/dbenders1/mavros/blob/RPYrT_support/mavros/src/plugins/setpoint_raw.cpp#L282). The rest of the communication chain is the same as for the attitude target.

*Note: in this template, switching between the different commands is possible using ROS dynamic reconfiguration (see *cfg/controller.cfg* for the definition of all reconfigurable parameters. They are processed in the `reconfigure_callback` function inside *src/py_ext_control_template/controller.py*).*

*Note 2: to complete a full experiment (i.e., from taking off to landing), please select a variant of the position control options, since they are able to reach the goal position. The other control options are meant to show how to use these commands rather than moving to a goal location.*


### Position control
Create the publisher:
```
self.pos_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
self.pos_target_msg.header.stamp = rospy.Time.now()
self.pos_t arget_msg.type_mask = 3576  # binary: 0000 1101 1111 1000 => ignore everything except position setpoints
self.pos_target_msg.position.x = 1
self.pos_target_msg.position.y = 1
self.pos_target_msg.position.z = 2
```
and publish:
```
self.pos_pub.publish(self.pos_target_msg)
```


### Position and yaw control
Create the publisher:
```
self.pos_yaw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
self.pos_target_msg.header.stamp = rospy.Time.now()
self.pos_target_msg.type_mask = 2552  # binary: 0000 1001 1111 1000 => ignore everything except position and yaw setpoints
self.pos_target_msg.position.x = 1
self.pos_target_msg.position.y = 1
self.pos_target_msg.position.z = 2
self.pos_target_msg.yaw = np.pi
```
and publish:
```
self.pos_yaw_pub.publish(self.pos_target_msg)
```

*Note: this type of control is currently not supported, since the yaw rate setpoint of the attitude controller gets overwritten by the yaw rate command to allow PX4 to execute RPYrT commands (see below).*


### Position and yaw rate control
Create the publisher:
```
self.pos_yawrate_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
self.pos_target_msg.header.stamp = rospy.Time.now()
self.pos_target_msg.type_mask = 1528  # binary: 0000 0101 1111 1000 => ignore everything except position and yaw rate setpoints
self.pos_target_msg.position.x = 1
self.pos_target_msg.position.y = 1
self.pos_target_msg.position.z = 2
self.pos_target_msg.yaw_rate = 1
```
and publish:
```
self.pos_yawrate_pub.publish(self.pos_target_msg)
```


### Velocity control
Create the publisher:
```
self.vel_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
```
fill the [mavros_msgs/PositionTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html) message:
```
self.pos_target_msg.header.stamp = rospy.Time.now()
self.pos_target_msg.type_mask = 3527  # binary: 0000 1101 1100 0111 => ignore everything except velocity setpoints
self.pos_target_msg.velocity.x = 0.5*cos_val
self.pos_target_msg.velocity.y = 0.5*sin_val
self.pos_target_msg.velocity.z = 0
```
and publish:
```
self.vel_pub.publish(self.pos_target_msg)
```


### Attitude control
Create the publisher:
```
self.att_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
```
fill the [mavros_msgs/AttitudeTarget](https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/AttitudeTarget.html) message:
```
# Define orientation in ZYX Euler angles (rad) and convert to quaternion
roll = np.pi/10 * sin_val
pitch = np.pi/10 * cos_val
yaw = 0.0
# axes = sxyz or rzyx are equivalent
# q = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
q = quaternion_from_euler(yaw, pitch, roll, axes='rzyx')

# Define thrust and convert to input desired by PX4: [0.1, 0.9] (assuming linear relation)
# Used for attitude commands
desired_thrust = 9.81
thrust = (desired_thrust - 9.81) / 22.629 + 0.674

# Fill and send message
self.att_target_msg.header.stamp = rospy.Time.now()
self.att_target_msg.type_mask = 63  # binary: 0011 1111 => ignore everything except attitude and thrust setpoints
self.att_target_msg.orientation.x = q[0]
self.att_target_msg.orientation.y = q[1]
self.att_target_msg.orientation.z = q[2]
self.att_target_msg.orientation.w = q[3]
self.att_target_msg.thrust = thrust
```
and publish:
```
self.att_pub.publish(self.att_target_msg)
```
*Note: the thrust value provided to PX4 should be given as a value between 0 and 1 (see [this link](https://docs.px4.io/master/en/concept/mixing.html#multirotor-mixer) and [this code line](https://github.com/dbenders1/mavros/blob/5c38353bea11af5139d81dc71d5d27df77b4df4f/mavros/src/plugins/setpoint_raw.cpp#L259)), because the real thrust the drone experiences depends on the specific hardware present on the platform (ESCs and motors). Therefore, before controlling the drone with a thrust value, first make sure to derive the relation between real thrust and commanded thrust. This relation depends on the battery voltage. This constraint on the commanded thrust holds for all messages containing a thrust value, including the RPYrT control message.*


### RPYrT control
Create the publisher:
```
self.rpyrt_pub = rospy.Publisher("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, queue_size=10)
```
fill the [mav_msgs/RollPitchYawrateThrust](https://docs.ros.org/en/indigo/api/mav_msgs/html/msg/RollPitchYawrateThrust.html) message:
```
# Define attitude (rad) and yaw rate (rad/s) commands
roll = np.pi/10 * sin_val
pitch = np.pi/10 * cos_val
yaw_rate = 1.0 

# Define thrust and convert to input desired by PX4: [0.1, 0.9] (assuming linear relation)
# Used for attitude commands
desired_thrust = 9.81
thrust = (desired_thrust - 9.81) / 22.629 + 0.674

# Fill and send message
self.rpyrt_msg.header.stamp = rospy.Time.now()
self.rpyrt_msg.roll = roll
self.rpyrt_msg.pitch = pitch
self.rpyrt_msg.yaw_rate = yaw_rate
self.rpyrt_msg.thrust.x = float('NaN')
self.rpyrt_msg.thrust.y = float('NaN')
self.rpyrt_msg.thrust.z = thrust
```
and publish:
```
self.rpyrt_pub.publish(self.rpyrt_msg)
```
