## Controller interface for drone toolbox in python
## by Max Lodel, TU Delft, 2022

import rospy
from drone_toolbox_ext_control_template.cfg import ControllerConfig


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

    def initialize(self) -> bool:
        pass

    def initRosInterface(self) -> bool:
        # Enable debug printing at least for the first part of the algorithm
        self.enable_debug = True

        # Store ROS parameters
        if not self.getRosParameters():
            #error message
            return False
        
        # Safety check: remove in case it is desired to test the controller in the lab
        if not self.is_sim:
            # error message
            exit(1)

        # Ros subscribers and publishers

        
    def getRosParameters(self) -> bool:
        # Required parameters
        if rospy.has_param('goal_pos'):
            self.goal_pos = rospy.get_param('goal_pos')
        else:
            return False
        
        if rospy.has_param('enable_debug'):
            self.enable_debug = rospy.get_param('enable_debug')
        else:
            return False

        # Non required
        self.is_sim = rospy.get_param('is_sim', self.is_sim_default)
        self.loop_frequency = rospy.get_param('loop_frequency', self.loop_frequency_default)
        self.dist_thres = rospy.get_param('dist_thres', self.dist_thres_default)

    def reconfigureCallback(self, config: ControllerConfig, level: int) -> None:
        pass