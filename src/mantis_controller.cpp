#include "drone_toolbox_ext_control_template/controller.h"
#include <torch/torch.h>
#include <thread>
#include <mavros_msgs/ActuatorControl.h>
#include <rosgraph_msgs/Clock.h>
#include <mavros_msgs/HoverThrustEstimate.h>

/************************************ INIT FUNCTIONS ************************************/
torch::Device cuda_device(torch::kCUDA);

std::vector<float> Controller::Create_Network_Output_landing(torch::Device cuda) {
    inputs_landing.emplace_back(torch::tensor({{cur_pos_[0], cur_pos_[2], cur_vel_[0], cur_vel_[2], cur_pitch}}).to(cuda));
    //CONTROLLER_INFO("input landing: " << cur_pos_[0] << " " << cur_pos_[2] << " " << cur_vel_[0] << " " << cur_vel_[2] << " " << cur_pitch << " " << cur_ngular_vel_[1]);
    // Use the forward function to get the policy output in an Ivalue object
    auto output_landing = module_landing.forward(inputs_landing);
    // Must convert Ivalue to a Tuple and then to a Tensor before processing to Float
    // Adopted from https://g-airborne.com/bringing-your-deep-learning-model-to-production-with-libtorch-part-2-tracing-your-pytorch-model/
    auto output_tuple_landing = output_landing.toTuple()->elements()[0].toTensor();
    auto output_size_landing = output_tuple_landing.sizes()[1];
    auto output_vector_landing = std::vector<float>(output_size_landing);
    for (int i = 0; i < output_size_landing; i++) {
    output_vector_landing[i] = output_tuple_landing[0][i].item<float>();
    }
    inputs_landing.clear();
    return output_vector_landing;
}

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
    control_method_ = ATT_CTRL;

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
    
    for (int i = 0; i < 5; i++) {    
        std::vector<float> test1 = Create_Network_Output_landing(cuda_device);
        CONTROLLER_INFO("test1: " << test1[0] << " " << test1[1]);
    }

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
    
    localization_method = ODOMETRY;

    // ROS subscribers and publishers
    switch (localization_method) {
        case GROUND_TRUTH:
            //vision_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1, &Controller::visionPoseCallback, this);
            ground_truth_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &Controller::gazeboStateCallback, this);
            break;
        case ESTIMATED:
            estimated_state_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/marker_pose", 1, &Controller::estimatedStateCallback, this);
            break;
        case ODOMETRY:
            odometry_sub = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &Controller::OdometryCallback, this);
            check_landing_sub = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &Controller::checkLandingCallback, this);
            break;    
        default:
            // Handle the case when localization_method is not recognized
            break;
    }

    pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pos_yaw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pos_yawrate_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    rpyrt_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", 1);
    actuator_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
    clock_sub = nh_.subscribe<rosgraph_msgs::Clock>("/clock", 1, &Controller::clockCallback, this);
    hover_thrust_est_sub_ = nh_.subscribe<mavros_msgs::HoverThrustEstimate>("/mavros/hover_thrust_estimate", 1, &Controller::hoverThrustCallback, this);

    // ROS service servers and clients
    enable_control_server_ = nh_.advertiseService("/px4_ext_cont_enable", &Controller::enableControlCallback, this);
    disable_control_server_ = nh_.advertiseService("/px4_ext_cont_disable", &Controller::disableControlCallback, this);
    mission_finished_client_ = nh_.serviceClient<std_srvs::Trigger>("/px4_mission_finished_ext_cont");

    return true;
        
        
}
//clockcallback
void Controller::clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
    simTime = msg->clock.toSec();
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
    double sin_val = sin(2*M_PI*f*t)*0.25;
    double cos_val = cos(2*M_PI*f*t);

    // Calculate and publish control commands
    switch (control_method_) {
        case POS_CTRL:
            pos_target_msg_.header.stamp = ros::Time::now();
            pos_target_msg_.type_mask = 0; //binary: 0000 1101 1111 1000 => ignore everything except position setpoints
            pos_target_msg_.position.x = 1;
            pos_target_msg_.position.y = 1;
            pos_target_msg_.position.z = 10;

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
            
            tf2Scalar roll = 0, pitch = 0, yaw = 0; 
            tf2::Quaternion q;
            double thrust = 0;//(desired_thrust - 9.81) / 22.629 + 0.674;
            std::string experiment = "DRL model";

            if (experiment == "random_flight") {
                double thrust_min = 0.45;
                double thrust_max = 0.53;
                double pitch_min = -M_PI / 9; // -pi/6
                double pitch_max = M_PI / 9;  // pi/6

                if (t - prev_time_random_flight > 1) {
                thrust_random = (thrust_min + static_cast<double>(std::rand()) / RAND_MAX * (thrust_max - thrust_min));
                pitch_random = (pitch_min + static_cast<double>(std::rand()) / RAND_MAX * (pitch_max - pitch_min));
                ROS_INFO("thrust_random: %f, pitch_random: %f", thrust_random, pitch_random);
                prev_time_random_flight = t;
                }
                q.setRPY(0,pitch_random,0);

                att_target_msg_.header.stamp = ros::Time::now();
                att_target_msg_.type_mask = 7; 
           
                att_target_msg_.orientation.x = q.x();
                att_target_msg_.orientation.y = q.y();
                att_target_msg_.orientation.z = q.z();
                att_target_msg_.orientation.w = q.w();
                att_target_msg_.thrust = thrust_random;

            
            att_pub_.publish(att_target_msg_);

            }
            if (experiment == "z_drag") {
                q.setRPY(0,0,0);

                att_target_msg_.header.stamp = ros::Time::now();
                att_target_msg_.type_mask = 7; 
           
                att_target_msg_.orientation.x = q.x();
                att_target_msg_.orientation.y = q.y();
                att_target_msg_.orientation.z = q.z();
                att_target_msg_.orientation.w = q.w();
                att_target_msg_.thrust = thrust;

            }
            else if(experiment == "x_drag"){
                pitch = (M_PI/30)*2;
                q.setRPY(0,pitch,0);
                thrust = 0.57;
                att_target_msg_.header.stamp = ros::Time::now();
                att_target_msg_.type_mask = 7; 
                att_target_msg_.orientation.x = q.x();
                att_target_msg_.orientation.y = q.y();
                att_target_msg_.orientation.z = q.z();
                att_target_msg_.orientation.w = q.w();
                att_target_msg_.thrust = thrust;
                
                
            }
            else if(experiment =="horizontal_flying"){
                
                att_target_msg_.header.stamp = ros::Time::now();
                att_target_msg_.type_mask = 7;
                att_target_msg_.orientation.x = q.x();
                att_target_msg_.orientation.y = q.y();
                att_target_msg_.orientation.z = q.z();
                att_target_msg_.orientation.w = q.w();
                att_target_msg_.thrust = thrust;
                

            }
            else if (experiment == "pitch_block_wave") {

                if ((t >= 1.0) && (t < 2)) {

                    pitch = (M_PI/30)*2;
                } 
                else {
                    pitch = 0.0;
                }

                q.setRPY(roll, pitch, yaw);
                att_target_msg_.header.stamp = ros::Time::now();
                att_target_msg_.type_mask = 7; 
                att_target_msg_.orientation.x = q.x();
                att_target_msg_.orientation.y = q.y();
                att_target_msg_.orientation.z = q.z();
                att_target_msg_.orientation.w = q.w();
                att_target_msg_.thrust = 0.5;
            }
            else if(experiment == "pitch_sine_wave"){
                double pitchOffset = 0.0; 
                double pitchAmplitude = (M_PI / 30) * 2.5;   
                pitch = pitchAmplitude * sin_val + pitchOffset;
                q.setRPY(roll, pitch, yaw);
                att_target_msg_.header.stamp = ros::Time::now();
                att_target_msg_.type_mask = 7; 
                att_target_msg_.orientation.x = q.x();
                att_target_msg_.orientation.y = q.y();
                att_target_msg_.orientation.z = q.z();
                att_target_msg_.orientation.w = q.w();
                att_target_msg_.thrust = 0.51;
                
                
               
            }
            else if(experiment =="actuator control") {
                ROS_INFO("actuator control");
                actuator_msg.header.stamp = ros::Time::now();
                actuator_msg.group_mix = 0;
                actuator_msg.controls[0] = 0;
                actuator_msg.controls[1] = 0;
                actuator_msg.controls[2] = 0;
                actuator_msg.controls[3] = 1;
                actuator_msg.controls[4] = 0;
                actuator_msg.controls[5] = 0;
                actuator_msg.controls[6] = 0;
                actuator_msg.controls[7] = 0;
                actuator_pub_.publish(actuator_msg);
            
            }  
            else if(experiment =="DRL model")
            {
                if (t < 0){
                    if (first_input == true){
                        first_input = false;
                        first_pos_ = cur_pos_;
                    }
                
                    pos_target_msg_.header.stamp = ros::Time::now();
                    pos_target_msg_.type_mask = 0; //binary: 0000 1101 1111 1000 => ignore everything except position setpoints
                    pos_target_msg_.position.x = first_pos_[0];
                    pos_target_msg_.position.y = first_pos_[1];
                    pos_target_msg_.position.z = first_pos_[2];
                    pos_target_msg_.yaw = 0;
                    pos_pub_.publish(pos_target_msg_);

                }
                else {
                if ((reached_goal)  || end_loop)  {
                    if (!end_loop)  {
                        CONTROLLER_INFO("at target position");
                        q.setRPY(0,cur_pitch,0);
                    }
                    // Fill and send message
                    att_target_msg_.header.stamp = ros::Time::now();
                    att_target_msg_.type_mask = 7; //binary: 0011 1111 => ignore everything except attitude and thrust setpoints
                    //att_target_msg_.orientation.x = first_pose_.orientation.x;
                    att_target_msg_.orientation.x = q.x();
                    att_target_msg_.orientation.y = q.y();
                    att_target_msg_.orientation.z = q.z();
                    att_target_msg_.orientation.w = q.w();
                    att_target_msg_.thrust = 0;
                    end_loop = true;
                }
    
                else {
                    std::vector<float> output_vector_landing = Create_Network_Output_landing(cuda_device);
                    
                    pitch = std::clamp(output_vector_landing[1], lower_bound, upper_bound)*M_PI/6;
                    float action_thrust = std::clamp(output_vector_landing[0], lower_bound, upper_bound)*5 + 9.81;
                    //ROS_INFO("thrust: %f, pitch: %f", action_thrust, pitch);
                    //float action_thrust_final = (((static_cast<float>(action_thrust)*3000)+300)*(static_cast<float>(action_thrust)*3000+300)*0.000000391*4);
                    q.setRPY(0, pitch, 0);
                    //float action_thrust = 9.8066+1;
                    //if (t < 15) {
                    //acc_magnitude = 12;
                    //}
                    //if (t >= 15 && t < 20) {
                    //acc_magnitude = 10;
                    //}
                    //if (t >= 20) {
                    acc_magnitude = action_thrust;
                    //}
                    
                    Eigen::AngleAxisd rollAngle(cur_roll, Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitchAngle(cur_pitch, Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yawAngle(cur_yaw, Eigen::Vector3d::UnitZ());           
                    Eigen::Quaterniond q_acc_magnitude = yawAngle * pitchAngle * rollAngle;
                    Eigen::Vector3d acc_vector_world = q_acc_magnitude * Eigen::Vector3d(0, 0, 1)*acc_magnitude;
                    //Eigen::Vector3d acc_vector_world = Eigen::Vector3d(0,0, 11);
                    acc_vector_world(2) -= g;                    
                    Eigen::Vector3d body_z = Eigen::Vector3d(acc_vector_world(0), acc_vector_world(1), g).normalized();
                    double collective_thrust = thrust_scaling_ * (acc_vector_world(2) * (hover_thrust_est_ / g)) + hover_thrust_est_;
                    collective_thrust /= Eigen::Vector3d(0,0,1).dot(body_z);
                    Eigen::Vector3d thrust_sp = body_z * collective_thrust;
                    Eigen::Matrix3d R_b_to_w = q_acc_magnitude.toRotationMatrix();
                    Eigen::Matrix3d R_w_to_b = R_b_to_w.transpose();
                    Eigen::Vector3d thrust_sp_body = R_w_to_b * Eigen::Vector3d(0, 0, 1);
                    double scalar = thrust_sp_body(2) / thrust_sp(2);
                    thrust_sp_body = thrust_sp_body / scalar;
                    double thrust_sp_body_scalar = sqrt(pow(thrust_sp_body(0), 2) + pow(thrust_sp_body(1), 2) + pow(thrust_sp_body(2), 2));
                    
                    att_target_msg_.header.stamp = ros::Time::now();
                    att_target_msg_.type_mask = 7; 

                    att_target_msg_.orientation.x = q.x();
                    att_target_msg_.orientation.y = q.y();
                    att_target_msg_.orientation.z = q.z();
                    att_target_msg_.orientation.w = q.w();
                    att_target_msg_.thrust = thrust_sp_body_scalar;
                    
                }
                att_pub_.publish(att_target_msg_);


                }
            
            }
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
bool Controller::checkPositionCondition(vector<double> cur_pos) {
    double threshold_pos_x = 0.35;
    double threshold_pos_y = 1.5;
    double threshold_pos_z = 0.15;
    double platform_x = 0;
    double platform_y = 0;
    double platform_z = 1.25;

    return (std::abs(cur_pos[0] - platform_x) < threshold_pos_x) &&
           (std::abs(cur_pos[1] - platform_y) < threshold_pos_y) &&
           (std::abs(cur_pos[2] - platform_z) < threshold_pos_z);
}

bool Controller::checkPitchCondition(double cur_pitch) {
    double platform_pitch = -0.44879895051;
    //double platform_pitch = -0.26179938779;
    double threshold_pitch = 0.1;//0.3;
    
    return std::abs(cur_pitch - platform_pitch) < threshold_pitch;
}

void Controller::estimatedStateCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {   
    ROS_INFO("receiving estimated_state information");

    if (!first_state_received_) {
        first_pose_ = msg.pose.pose;
        first_pose_.position.x -= 4.5;
        first_state_received_ = true;
    }

    tf::Quaternion q1(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);

    cur_pose_ = msg.pose.pose;
    cur_pose_.position.x -= 4.5;
    cur_pos_ = {cur_pose_.position.x, cur_pose_.position.y, cur_pose_.position.z};
    tf::Matrix3x3 m(q1);
    m.getRPY(cur_roll, cur_pitch, cur_yaw);

    if (std::isnan(cur_pitch)) {
        cur_pitch = prev_pitch;
    }
    prev_pitch = cur_pitch;
    
    pitchCondition = checkPitchCondition(cur_pitch);
    posCondition = checkPositionCondition(cur_pos_);

    if (pitchCondition && posCondition) {
        reached_goal = true;
        ROS_INFO("reached goal in mantis controller");
    }
    ros::Time cur_time = ros::Time::now();
    double delta_t = cur_time.toSec() - prev_time; // Time interval between position estimations
    prev_time = cur_time.toSec();

    double delta_x = cur_pos_[0] - prev_pos_[0];
    double vel_x = delta_x / delta_t;
    double delta_y = cur_pos_[1] - prev_pos_[1];
    double vel_y = delta_y / delta_t;
    double delta_z = cur_pos_[2] - prev_pos_[2];
    double vel_z = delta_z / delta_t;
    cur_vel_ = {vel_x, vel_y, vel_z};
    prev_pos_ = cur_pos_;
    
}
void Controller::visionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {   
    ROS_INFO("receiving vision_pose information");

    if (!first_state_received_) {
        first_pose_ = msg->pose;
        first_state_received_ = true;
    }

    tf::Quaternion q1(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    cur_pose_ = msg->pose;
    cur_pos_ = {cur_pose_.position.x, cur_pose_.position.y, cur_pose_.position.z};
    tf::Matrix3x3 m(q1);
    m.getRPY(cur_roll, cur_pitch, cur_yaw);
    pitchCondition = checkPitchCondition(cur_pitch);
    posCondition = checkPositionCondition(cur_pos_);

    if (pitchCondition && posCondition) {
        reached_goal = true;
        ROS_INFO("reached goal in mantis controller");
    }
}
void Controller::gazeboStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {   
    ROS_INFO("receiving gt information");
    cur_ground_truth_ = *msg;

    int mantisIndex = -1;
    for (size_t i = 0; i < cur_ground_truth_.name.size(); ++i)
    {
        if (cur_ground_truth_.name[i] == "mantis" || cur_ground_truth_.name[i] == "nxp_drone" || cur_ground_truth_.name[i] == "agile")
        {
            mantisIndex = static_cast<int>(i);
            break;
        }
    }
    
    if (mantisIndex != -1 && mantisIndex < cur_ground_truth_.pose.size())
    {
        cur_pose_ = cur_ground_truth_.pose[mantisIndex];
    
    if (!first_state_received_) {
        first_pose_ = cur_pose_;
        first_state_received_ = true;
    }
    tf::Quaternion q1(
        cur_pose_.orientation.x,
        cur_pose_.orientation.y,
        cur_pose_.orientation.z,
        cur_pose_.orientation.w);

    cur_pos_ = {cur_pose_.position.x, cur_pose_.position.y, cur_pose_.position.z};
    
    tf::Matrix3x3 m(q1);
    m.getRPY(cur_roll, cur_pitch, cur_yaw);
    pitchCondition = checkPitchCondition(cur_pitch);
    posCondition = checkPositionCondition(cur_pos_);

    if (pitchCondition && posCondition) {
        reached_goal = true;
        ROS_INFO("reached goal in mantis controller");
    }

    cur_vel_ = {cur_ground_truth_.twist[mantisIndex].linear.x, cur_ground_truth_.twist[mantisIndex].linear.y, cur_ground_truth_.twist[mantisIndex].linear.z};
    cur_angular_vel_ = {cur_ground_truth_.twist[mantisIndex].angular.x, cur_ground_truth_.twist[mantisIndex].angular.y, cur_ground_truth_.twist[mantisIndex].angular.z};
    }
    else
    {
        CONTROLLER_ERROR("Model with name 'mantis' not found in gazebo model states message!");
    }
}

void Controller::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {   
    if (!first_state_received_) {
        first_state_received_ = true;
    }

    cur_odom_ = *msg;
    cur_pos_ = {cur_odom_.pose.pose.position.x-4.5, cur_odom_.pose.pose.position.y, cur_odom_.pose.pose.position.z};
    cur_vel_ = {cur_odom_.twist.twist.linear.x, cur_odom_.twist.twist.linear.y, cur_odom_.twist.twist.linear.z};
    tf::Quaternion q1(
        cur_odom_.pose.pose.orientation.x,
        cur_odom_.pose.pose.orientation.y,
        cur_odom_.pose.pose.orientation.z,
        cur_odom_.pose.pose.orientation.w);
    tf::Matrix3x3 m_odom(q1);
    m_odom.getRPY(cur_roll, cur_pitch, cur_yaw);
    
    //pitchCondition = checkPitchCondition(cur_pitch);
    //posCondition = checkPositionCondition(cur_pos_);
    //if (pitchCondition) {
    //    ROS_INFO("pitch condition met");
    //}
    //if (posCondition) {
    //    ROS_INFO("pos condition met");
    //}
    //if (pitchCondition && posCondition) {
    //    reached_goal = true;
    //    ROS_INFO("reached goal in mantis controller");

    //}

}

void Controller::checkLandingCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    /*
    cur_ground_truth_ = *msg;

    int mantisIndex = -1;
    for (size_t i = 0; i < cur_ground_truth_.name.size(); ++i)
    {
        if (cur_ground_truth_.name[i] == "mantis" || cur_ground_truth_.name[i] == "nxp_drone" || cur_ground_truth_.name[i] == "agile")
        {
            mantisIndex = static_cast<int>(i);
            break;
        }
    }
    
    if (mantisIndex != -1 && mantisIndex < cur_ground_truth_.pose.size())
    {
        gazebo_pose_ = cur_ground_truth_.pose[mantisIndex];
    
    if (!first_state_received_) {
        first_pose_ = cur_pose_;
        first_state_received_ = true;
    }
    tf::Quaternion q1(
        gazebo_pose_.orientation.x,
        gazebo_pose_.orientation.y,
        gazebo_pose_.orientation.z,
        gazebo_pose_.orientation.w);

    gazebo_pos_ = {gazebo_pose_.position.x, gazebo_pose_.position.y, gazebo_pose_.position.z};
    
    tf::Matrix3x3 m_gazebo(q1);
    m_gazebo.getRPY(gazebo_roll, gazebo_pitch, gazebo_yaw);
    */
    pitchCondition = checkPitchCondition(cur_pitch);
    posCondition = checkPositionCondition(cur_pos_);

    if (pitchCondition && posCondition) {
        reached_goal = true;
        ROS_INFO("reached goal in mantis controller");
    }

    //cur_vel_ = {cur_ground_truth_.twist[mantisIndex].linear.x, cur_ground_truth_.twist[mantisIndex].linear.y, cur_ground_truth_.twist[mantisIndex].linear.z};
    //cur_angular_vel_ = {cur_ground_truth_.twist[mantisIndex].angular.x, cur_ground_truth_.twist[mantisIndex].angular.y, cur_ground_truth_.twist[mantisIndex].angular.z};
    //}
    //else
    //{
    //    CONTROLLER_ERROR("Model with name 'mantis' not found in gazebo model states message!");
    //}
}
void Controller::hoverThrustCallback(const mavros_msgs::HoverThrustEstimate::ConstPtr& msg)
{   //ROS_INFO("hovergames core controller");
    hover_thrust_est_valid_ = msg->valid;
    if (hover_thrust_est_valid_) {
        //ROS_INFO("hover thrust estimate valid: %f", msg->hover_thrust);
        hover_thrust_est_ = msg->hover_thrust;
    }
}

/****************************************************************************************/
