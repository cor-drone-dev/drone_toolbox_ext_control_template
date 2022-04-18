#ifndef CONTROLLER
#define CONTROLLER

// General include
#include <math.h>

// ROS includes
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <drone_toolbox_ext_control_template/ControllerConfig.h>

#include <geometry_msgs/Pose.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_srvs/Trigger.h>

// Custom includes
#include "drone_toolbox_ext_control_template/helpers.h"

using namespace std;

class Controller
{
  public:
    // Constructor and destructor
    Controller(const ros::NodeHandle &nh) : nh_(nh) {};
    ~Controller() {};

    // Class initialization
    bool initialize();

    // Dynamic reconfiguration
    void reconfigureCallback(drone_toolbox_ext_control_template::ControllerConfig& config, uint32_t level);

  private:
    // Init functions
    bool initRosInterface();
    bool getRosParameters();

    // Loop functions
    void loop(const ros::TimerEvent &event);
    void missionFinishedCheck();
    void controllerExecution();

    // Callback functions
    bool enableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool disableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void stateCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // ROS parameter functions
    // Return false if non-existent
    template <typename T>
    bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
    {
      if (!nh.getParam(name, value)) {
        CONTROLLER_WARN("Parameter " << name << " not set.");
        return false;
      } else {
        return true;
      }
    }
    // Use default if non-existent
    template <typename T>
    void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
    {
      if (!retrieveParameter(nh, name, value)) {
        CONTROLLER_WARN("Setting " << name << " to default value: " << default_value << ".");
        value = default_value;
      }
    }

    // Indicate experiment or simulation
    bool is_sim_;
    const bool is_sim_default_ = false;

    // ROS nodehandle
    ros::NodeHandle nh_;

    // ROS subscribers and publishers
    ros::Subscriber state_sub_;
    ros::Publisher pos_pub_, pos_yaw_pub_, pos_yawrate_pub_, vel_pub_, att_pub_;

    // ROS service servers and clients
    ros::ServiceServer enable_control_server_, disable_control_server_;
    ros::ServiceClient mission_finished_client_;

    // ROS messages
    // Send
    mavros_msgs::PositionTarget pos_target_msg_;
    mavros_msgs::AttitudeTarget att_target_msg_;
    // Receive
    geometry_msgs::Pose cur_pose_;
    nav_msgs::Odometry cur_odom_;

    // ROS services
    std_srvs::Trigger mission_finished_srv_;

    // ROS dynamic reconfiguration
    bool first_reconfig_cb_;
    bool enable_debug_;

    // Mission-related
    bool first_state_received_;
    double dist_thres_;
    const double dist_thres_default_ = 0.1;

    // Time declarations
    double loop_frequency_;
    const double loop_frequency_default_ = 20;
    ros::Timer loop_timer_;
    bool timer_running_;
    ros::Time loop_start_time_;

    // 3D lab/experiment settings
    vector<double> cur_pos_{vector<double>(3,0)};
    vector<double> goal_pos_{vector<double>(3,0)};

    // Control method
    enum ControlMethod {
      POS_CTRL,
      POS_YAW_CTRL,
      POS_YAWRATE_CTRL,
      VEL_CTRL,
      ATT_CTRL
    } control_method_;
};

#endif //CONTROLLER
