import rospy 

from dynamic_reconfigure.server import Server
from drone_toolbox_ext_control_template.cfg import ControllerConfig

from py_ext_control_template import Controller

if __name__ == '__main__':

    try:
        rospy.init_node('py_controller', anonymous=True)

        controllerObj = Controller()

        server = Server(ControllerConfig, controllerObj.reconfigure_callback)

        if not controllerObj.initialize():
            rospy.logerr('[CONTROLLER]: Controller initialization failed!')
            # exit(1)
        else:
            rospy.loginfo('[CONTROLLER]: Controller initialization successful')
            rospy.spin()

    except:
        # exception = rospy.exceptions
        rospy.logerr('[CONTROLLER]: Error occurred: ')
        exit(1)
