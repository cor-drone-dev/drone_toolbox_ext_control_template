#include "drone_toolbox_ext_control_template/controller.h"

int main(int argc, char **argv)
{
  try {
    // Initialize ROS node
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("controller");

    // Create PX4 control interface instance
    Controller* controller = new Controller(nh);

    // Create dynamic reconfiguration server
    dynamic_reconfigure::Server<drone_toolbox_ext_control_template::ControllerConfig> server(nh);
    dynamic_reconfigure::Server<drone_toolbox_ext_control_template::ControllerConfig>::CallbackType f;
    f = boost::bind(&Controller::reconfigureCallback, controller, _1, _2);
    server.setCallback(f);

    // Initialize controller and catch possible errors
    if (!controller->initialize()) {
      CONTROLLER_ERROR("Controller initialization failed!");
      exit(1);
    } else {
      CONTROLLER_INFO_ALWAYS("Controller initialization successful");
      ros::spin();
    }
  } catch (ros::Exception& e) {
    CONTROLLER_ERROR_STREAM("Error occurred: " << e.what() << "!");
    exit(1);
  }

  return 0;
}
