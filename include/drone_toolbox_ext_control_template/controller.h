#ifndef CONTROLLER
#define CONTROLLER

// General include
#include <math.h>
#include <tf/tf.h>

// ROS includes
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <drone_toolbox_ext_control_template/ControllerConfig.h>
#include <thread>
#include <mutex>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/ActuatorControl.h>
#include <torch/script.h>
#include <torch/torch.h>
#include <gazebo_msgs/ModelStates.h>
#include <rosgraph_msgs/Clock.h>
// Custom includes
#include "drone_toolbox_ext_control_template/helpers.h"
#include <Eigen/Dense>

#include <mavros_msgs/HoverThrustEstimate.h>

using namespace std;

class Controller
{
  public:

    // Constructor and destructor
    Controller(const ros::NodeHandle &nh) : nh_(nh), module_landing(torch::jit::load("/root/dev/catkin_ws/src/model/may27B10traced_model.pt"))
    { 
      module_landing.to(torch::kCUDA);
    };

    //Controller(const ros::NodeHandle &nh) : nh_(nh) {};
    ~Controller() {};

    // Class initialization
    bool initialize();

    // Dynamic reconfiguration
    void reconfigureCallback(drone_toolbox_ext_control_template::ControllerConfig& config, uint32_t level);
    //DRL
    torch::jit::script::Module module_landing;
    std::vector<torch::jit::IValue> inputs_landing;
    std::vector<float> output_vector_landing;
    std::vector<float> output_vector_setpoint;
    bool first_vio = false;
    bool first_input = true;
    double cur_roll, cur_yaw;
    double cur_pitch = 0;
    double gazebo_roll, gazebo_pitch, gazebo_yaw;
    float lower_bound = -1;
    float upper_bound = 1;
    double hover_thrust = 0.4;
    double thrust_radius = 0.3;
    double g = 9.8066;
  
    std::vector<float> Create_Network_Output_landing(torch::Device cuda);

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
    void gazeboStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void estimatedStateCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);
    void visionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);  
    bool checkPitchCondition(double cur_pitch);
    bool checkPositionCondition(vector<double> cur_pos);
    void checkLandingCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void hoverThrustCallback(const mavros_msgs::HoverThrustEstimate::ConstPtr& msg);

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
    // 3D lab/experiment settings
    vector<double> cur_pos_{vector<double>(3,0)};
    vector<double> first_pos_{vector<double>(3,0)};
    vector<double> cur_vel_{vector<double>(3,0)};
    vector<double> cur_angular_vel_{vector<double>(3,0)};
    vector<double> goal_pos_{vector<double>(3,0)};
    vector<double> gazebo_pos_{vector<double>(3,0)};
    
    double thrust_scaling_ = 0.67;
    //double hover_thrust_ = 0.8;
    double thrust_random, pitch_random;
    double prev_time;
    double prev_time_random_flight = -10;
    double prev_pitch;
    ros::Time cur_time; 
    bool pitchCondition;
    bool posCondition;
    bool end_loop;
    bool reached_goal;
    vector<double> prev_pos_{vector<double>(3,0)};
    tf2::Quaternion q;

    bool hover_thrust_est_valid_;
    double hover_thrust_est_ = 0.472;
    double acc_magnitude = 10;

    // Indicate experiment or simulation
    bool is_sim_ = true;
    const bool is_sim_default_ = true;

    // ROS nodehandle
    ros::NodeHandle nh_;

    // ROS subscribers and publishers
    ros::Subscriber state_sub_, ground_truth_sub_, estimated_state_sub, odometry_sub, clock_sub, hover_thrust_est_sub_, vision_pose_sub_, check_landing_sub;
    ros::Publisher pos_pub_, pos_yaw_pub_, pos_yawrate_pub_, vel_pub_, att_pub_, rpyrt_pub_;
    ros::Publisher actuator_pub_;
    // ROS service servers and clients
    ros::ServiceServer enable_control_server_, disable_control_server_;
    ros::ServiceClient mission_finished_client_;

    // ROS messages
    // Send
    mavros_msgs::PositionTarget pos_target_msg_;
    mavros_msgs::AttitudeTarget att_target_msg_;
    mav_msgs::RollPitchYawrateThrust rpyrt_msg_;
    // Receive
    gazebo_msgs::ModelStates cur_ground_truth_;
    geometry_msgs::Pose cur_pose_;
    geometry_msgs::Pose gazebo_pose_;

    geometry_msgs::Pose first_pose_;
    nav_msgs::Odometry cur_odom_;

   mavros_msgs::ActuatorControl actuator_msg;
    // ROS services
    std_srvs::Trigger mission_finished_srv_;

    // ROS dynamic reconfiguration
    bool first_reconfig_cb_;
    bool enable_debug_;

    // Mission-related
    bool mission_finished_;
    bool first_state_received_;
    double dist_thres_;
    const double dist_thres_default_ = 0.1;

    // Time declarations
    double loop_frequency_;
    const double loop_frequency_default_ = 20;
    ros::Timer loop_timer_;
    bool timer_running_;
    ros::Time loop_start_time_, last_time_;
    ros::Duration print_timeout_;
    double simTime;
    enum LocalizationMethod {
        GROUND_TRUTH,
        ESTIMATED,
        ODOMETRY
    } localization_method;

    // Control method
    enum ControlMethod {
      POS_CTRL,
      POS_YAW_CTRL,
      POS_YAWRATE_CTRL,
      VEL_CTRL,
      ATT_CTRL,
      RPYRT_CTRL
    } control_method_;
};

#endif //CONTROLLER
