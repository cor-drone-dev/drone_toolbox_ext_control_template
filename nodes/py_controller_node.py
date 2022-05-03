import rospy 

from dynamic_reconfigure.server import Server
from drone_toolbox_ext_control_template.cfg import ControllerConfig

from py_ext_control_template import Controller

if __name__ == '__main__':

    try:
        rospy.init_node('py_controller', anonymous=True)

        controllerObj = Controller()

        server = Server(ControllerConfig, controllerObj.reconfigureCallback)

        if not controllerObj.initialize():
            # error message
            exit(1)
        else:
            # info message
            rospy.spin()

    except rospy.exceptions:
        # error message
        pass
