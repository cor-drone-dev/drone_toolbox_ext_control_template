## Controller interface for drone toolbox in python
## by Max Lodel, TU Delft, 2022

import enum
import numpy as np

import rospy
from drone_toolbox_ext_control_template.cfg import ControllerConfig

from mavros_msgs.msg import PositionTarget, AttitudeTarget
from mav_msgs.msg import RollPitchYawrateThrust
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from std_srvs.srv import Trigger

from py_ext_control_template.custom_timer import CustomTimer

from tf.transformations import quaternion_from_euler

class ControlMethod(enum.Enum):
    POS_CTRL = 0
    POS_YAW_CTRL = 1
    POS_YAWRATE_CTRL = 2
    VEL_CTRL = 3
    ATT_CTRL = 4
    RPYRT_CTRL = 5

class Controller:
    def __init__(self) -> None:
        
        # Required parameters
        self.enable_debug = None
        self.goal_pos = None

        # Non-required parameters
        self.is_sim = None
        self.is_sim_default = False
        self.loop_frequency = None
        self.loop_frequency_default = 20
        self.dist_thres = None
        self.dist_thres_default = 0.1
        

        # ROS publishers and subscribers
        self.state_sub = None
        self.pos_pub = None
        self.pos_yaw_pub = None
        self.pos_yawrate_pub = None
        self.vel_pub = None
        self.att_pub = None
        self.rpyrt_pub = None

        # ROS services and clients
        self.enable_control_srv = None
        self.disable_control_srv = None
        self.mission_finished_clt = None

        # ROS message/service variables
        self.pos_target_msg = PositionTarget()
        self.att_target_msg = AttitudeTarget()
        self.rpyrt_msg = RollPitchYawrateThrust()
        self.cur_pose_msg = Pose()
        self.cur_odom_msg = Odometry()
        self.mission_finished_srv = Trigger()

        # Convenience placeholder for converted messages
        self.cur_pos = np.zeros((3,1))

        # Controller variables
        self.mission_finished = None
        self.first_state_received = None
        self.control_method = None

        # Timer variables
        self.print_timeout = None
        self.timer_running = None
        self.loop_timer = None
        self.loop_start_time = None
        self.last_time = None

        self.first_reconfig_cb = None


    def initialize(self) -> bool:
        
        # Initialize ROS interface
        if not self._init_ros_interface():
            rospy.logerr('[CONTROLLER]: Failed to initialize ROS interface')
            return False
        
        # Initialize controller variables
        self.mission_finished = False
        self.first_state_received = False
        self.control_method = ControlMethod.RPYRT_CTRL
        # Set timeout for error printing
        self.print_timeout = rospy.Duration(1.0)

        # Construct default position target message (used for position, velocity, and yaw/yawrate commands)
        self.pos_target_msg.header.frame_id = "base_link"
        self.pos_target_msg.coordinate_frame = 1 # binary: 0000 1101 1111 1111 => ignore everything
        self.pos_target_msg.type_mask = 3583
        self.pos_target_msg.position.x = float('NaN')
        self.pos_target_msg.position.y = float('NaN')
        self.pos_target_msg.position.z = float('NaN')
        self.pos_target_msg.velocity.x = float('NaN')
        self.pos_target_msg.velocity.y = float('NaN')
        self.pos_target_msg.velocity.z = float('NaN')
        self.pos_target_msg.acceleration_or_force.x = float('NaN')
        self.pos_target_msg.acceleration_or_force.y = float('NaN')
        self.pos_target_msg.acceleration_or_force.z = float('NaN')
        self.pos_target_msg.yaw = float('NaN')
        self.pos_target_msg.yaw_rate = float('NaN')

        # Construct default attitude target message (used for attitude and thrust commands)
        self.att_target_msg.header.frame_id = "base_link"
        self.att_target_msg.type_mask = 255 # binary: 1111 1111 => ignore everything
        self.att_target_msg.orientation.x = float('NaN')
        self.att_target_msg.orientation.y = float('NaN')
        self.att_target_msg.orientation.z = float('NaN')
        self.att_target_msg.orientation.w = float('NaN')
        self.att_target_msg.body_rate.x = float('NaN')
        self.att_target_msg.body_rate.y = float('NaN')
        self.att_target_msg.body_rate.z = float('NaN')
        self.att_target_msg.thrust = float('NaN')

        # Define control loop timer at specified frequency: should be higher than 2Hz
        self.loop_timer = CustomTimer(rospy.Duration(1/self.loop_frequency), self.loop)
        self.loop_timer.pause()
        self.timer_running = False

        return True


    def _init_ros_interface(self) -> bool:
        # Enable debug printing at least for the first part of the algorithm
        self.enable_debug = True

        # Store ROS parameters
        if not self._get_ros_parameters():
            rospy.logerr('[CONTROLLER]: Failed to obtain ROS parameters!')
            return False
        
        # Safety check: remove in case it is desired to test the controller in the lab
        if not self.is_sim:
            rospy.logerr('[CONTROLLER]: Sim is set to false, but this is not allowed in this template controller!')
            exit(1)

        # Ros subscribers and publishers
        self.state_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.state_callback)

        self.pos_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.pos_yaw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.pos_yawrate_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.att_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
        self.rpyrt_pub = rospy.Publisher("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, queue_size=10)

        # ROS services and clients
        self.enable_control_srv = rospy.Service("/px4_ext_cont_enable", Trigger, self.enable_control_callback)
        self.disable_control_srv = rospy.Service("/px4_ext_cont_disable", Trigger, self.disable_control_callback)
        self.mission_finished_clt = rospy.ServiceProxy("/px4_mission_finished_ext_cont", Trigger)        

        return True


    def _get_ros_parameters(self) -> bool:
        # Required parameters
        if rospy.has_param('goal_pos'):
            self.goal_pos = rospy.get_param('goal_pos')
        else:
            rospy.logerr('goal_pos not found')
            return False
        
        if rospy.has_param('enable_debug'):
            self.enable_debug = rospy.get_param('enable_debug')
        else:
            return False

        # Non required
        self.is_sim = rospy.get_param('is_sim', self.is_sim_default)
        self.loop_frequency = rospy.get_param('loop_frequency', self.loop_frequency_default)
        self.dist_thres = rospy.get_param('dist_thres', self.dist_thres_default)

        return True


    def loop(self, timer_event: rospy.timer.TimerEvent) -> None:
        # Check if goal reached
        self.mission_finished_check()

        # Compute and publish control commands
        self.controller_execution()


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


    def controller_execution(self) -> None:
        '''
        Insert custom controller code here.
        Possibly create different classes for different controller parts to keep the code modular.
        For example, use multiple instances of a PID class to control thrust and x/y positions separately.

        The code below is provided as an example to show the different ways of interfacing with MAVROS and PX4
        '''

        # A standard sine and cosine wave for velocity and attitude commands
        cur_time = rospy.Time.now()
        f = 0.25
        t = cur_time.to_sec() - self.loop_start_time.to_sec()
        sin_val = np.sin(2*np.pi*f*t)
        cos_val = np.cos(2*np.pi*f*t)

        # Calculate and publish control commands
        # Example given for each control mode

        if self.control_method == ControlMethod.POS_CTRL:
            self.pos_target_msg.header.stamp = rospy.Time.now()
            self.pos_target_msg.type_mask = 3576  # binary: 0000 1101 1111 1000 => ignore everything except position setpoints
            self.pos_target_msg.position.x = 1
            self.pos_target_msg.position.y = 1
            self.pos_target_msg.position.z = 2

            # Publish message
            self.pos_pub.publish(self.pos_target_msg)
        
        elif self.control_method == ControlMethod.POS_YAW_CTRL:
            # TODO edit PX4 controller code to allow this interface to work properly
            self.pos_target_msg.header.stamp = rospy.Time.now()
            self.pos_target_msg.type_mask = 2552  # binary: 0000 1001 1111 1000 => ignore everything except position and yaw setpoints
            self.pos_target_msg.position.x = 1
            self.pos_target_msg.position.y = 1
            self.pos_target_msg.position.z = 2
            self.pos_target_msg.yaw = np.pi

            # Publish Message
            self.pos_yaw_pub.publish(self.pos_target_msg)

        elif self.control_method == ControlMethod.POS_YAWRATE_CTRL:
            self.pos_target_msg.header.stamp = rospy.Time.now()
            self.pos_target_msg.type_mask = 1528  # binary: 0000 0101 1111 1000 => ignore everything except position and yaw rate setpoints
            self.pos_target_msg.position.x = 1
            self.pos_target_msg.position.y = 1
            self.pos_target_msg.position.z = 2
            self.pos_target_msg.yaw_rate = 1

            # Publish Message
            self.pos_yawrate_pub.publish(self.pos_target_msg)

        elif self.control_method == ControlMethod.VEL_CTRL:
            self.pos_target_msg.header.stamp = rospy.Time.now()
            self.pos_target_msg.type_mask = 3527  # binary: 0000 1101 1100 0111 => ignore everything except velocity setpoints
            self.pos_target_msg.velocity.x = 0.5*cos_val
            self.pos_target_msg.velocity.y = 0.5*sin_val
            self.pos_target_msg.velocity.z = 0

            # Publish Message
            self.vel_pub.publish(self.pos_target_msg)

        elif self.control_method == ControlMethod.ATT_CTRL:
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

            # Publish Message
            self.att_pub.publish(self.att_target_msg)
            
        elif self.control_method == ControlMethod.RPYRT_CTRL:
            
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

            self.rpyrt_pub.publish(self.rpyrt_msg)

  
    def reconfigure_callback(self, config: ControllerConfig, level: int) -> ControllerConfig:
        
        if level == 1:
            if self.first_reconfig_cb:
                self.first_reconfig_cb = False
                config.enable_debug = self.enable_debug
            else:
                self.enable_debug = config.enable_debug

        if level == 2:

            if config.control_select == 0:
                rospy.loginfo('[CONTROLLER]: Switching to position control')
                self.control_method = ControlMethod.POS_CTRL

            elif config.control_select == 1:
                rospy.loginfo('[CONTROLLER]: Combined position and yaw control currently not available! Switching to position control')
                self.control_method = ControlMethod.POS_CTRL

            elif config.control_select == 2:
                rospy.loginfo('[CONTROLLER]: Switching to combined position and yaw rate control')
                self.control_method = ControlMethod.POS_YAWRATE_CTRL

            elif config.control_select == 3:
                rospy.loginfo('[CONTROLLER]: Switching to velocity control')
                self.control_method = ControlMethod.VEL_CTRL

            elif config.control_select == 4:
                rospy.loginfo('[CONTROLLER]: Switching to attitude control')
                self.control_method = ControlMethod.ATT_CTRL

            elif config.control_select == 5:
                rospy.loginfo('[CONTROLLER]: Switching to roll, pitch, yaw rate and thrust control')
                self.control_method = ControlMethod.RPYRT_CTRL

            else:
                rospy.logwarn('[CONTROLLER]: Invalid control method selected! Switching to position control')
                self.control_method = ControlMethod.POS_CTRL

        return config
    

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


    def disable_control_callback(self, req: Trigger._request_class) -> Trigger._response_class:
        
        res = Trigger._response_class()

        rospy.loginfo('[CONTROLLER]: disableControlCallback called: stopping control loop')
        self.loop_timer.pause()
        self.timer_running = False
        res.success = True
        res.message = "Disabled control loop"

        return res


    def state_callback(self, msg: Odometry) -> None:
        if not self.first_state_received:
            self.first_state_received = True

        self.cur_odom_msg = msg
        self.cur_pose_msg = self.cur_odom_msg.pose.pose

        self.cur_pos[0] = self.cur_pose_msg.position.x
        self.cur_pos[1] = self.cur_pose_msg.position.y
        self.cur_pos[2] = self.cur_pose_msg.position.z