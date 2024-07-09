#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include <fstream>
#include <sstream>
#include <deque>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>

class ControllerNode {
public:


    ControllerNode() : tfBuffer(), listener(tfBuffer) {

        markerSub = nh.subscribe("/aruco_marker_publisher/markers", 1, &ControllerNode::markerArrayCallback, this);
        ground_truth_sub_ = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &ControllerNode::gazeboStateCallback, this); 
        vio_pub = nh.advertise<nav_msgs::Odometry>("/vio_pose", 1);
        marker_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/marker_pose", 1);
        vio_odom_sub = nh.subscribe<nav_msgs::Odometry>("/ov_msckf/odomimu", 1, &ControllerNode::VioOdomCallback, this);
        ekf_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &ControllerNode::EkfCallback, this);
    }

    void VioOdomCallback(const nav_msgs::Odometry::ConstPtr& vioOdom) {
            geometry_msgs::TransformStamped mapToOdomTransform = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
            
            if (first_vio) {
                pos_x_error = markerPose.pose.pose.position.x;
                pos_z_error = markerPose.pose.pose.position.z;
                first_vio = false;
            }
            
            
            nav_msgs::Odometry vio_odom_pose_msg_;
            
            vio_odom_pose_msg_.header.frame_id = "odom";
            vio_odom_pose_msg_.child_frame_id = "base_link";
            vio_odom_pose_msg_.pose.pose = vioOdom->pose.pose;
            vio_odom_pose_msg_.pose.pose.position.x = -vio_odom_pose_msg_.pose.pose.position.x;            
            vio_odom_pose_msg_.pose.pose.position.y = ground_truth_msg_.pose.position.y;
            vio_odom_pose_msg_.twist.twist = vioOdom->twist.twist;
            vio_odom_pose_msg_.pose.pose.position.x += pos_x_error;
            vio_odom_pose_msg_.pose.pose.position.z += pos_z_error;
            vio_odom_pose_msg_.header.stamp = vioOdom->header.stamp;
            //get linear velocity from position messages instead:
            curr_timestamp = vioOdom->header.stamp.toSec();
            dt = curr_timestamp - prev_timestamp;
            prev_timestamp = curr_timestamp;
            curr_velocity_x = (vio_odom_pose_msg_.pose.pose.position.x - prev_pos_x) / dt;
            curr_velocity_z = (vio_odom_pose_msg_.pose.pose.position.z - prev_pos_z) / dt;
            //cap the velocity to have a maximum value of 5 m/s in case of outliers:
            if (curr_velocity_x > 5) {
                curr_velocity_x = 5;
            }
            if (curr_velocity_z > 5) {
                curr_velocity_z = 5;
            }
            //also cap to -5
            if (curr_velocity_x < -5) {
                curr_velocity_x = -5;
            }
            if (curr_velocity_z < -5) {
                curr_velocity_z = -5;
            }
            prev_pos_x = vio_odom_pose_msg_.pose.pose.position.x;
            prev_pos_z = vio_odom_pose_msg_.pose.pose.position.z;
            vio_odom_pose_msg_.twist.twist.linear.x = curr_velocity_x;
            vio_odom_pose_msg_.twist.twist.linear.z = curr_velocity_z;
            //
            tf2::fromMsg(vio_odom_pose_msg_.pose.pose.orientation, q_vio_callback);
            tf2::Matrix3x3(q_vio_callback).getRPY(roll, pitch, yaw);
            tf2::Quaternion VioCallbackOrientation;

            //set the pitch velocity based on pitch estimations
            pitch_vel = (pitch - prev_pitch) / dt;
            prev_pitch = pitch;
            vio_odom_pose_msg_.twist.twist.angular.y = pitch_vel;

            vio_odom_pose_msg_.pose.covariance = vioOdom->pose.covariance;
            vio_odom_pose_msg_.pose.covariance = {1, 0, 0, 0, 0, 0,
                                        0, 1, 0, 0, 0, 0,
                                        0, 0, 1, 0, 0, 0,
                                        0, 0, 0, 1e-10, 0, 0,
                                        0, 0, 0, 0, 1e-10, 0,
                                        0, 0, 0, 0, 0, 1e-10};
            vio_odom_pose_msg_.twist.covariance = {1e-10, 0, 0, 0, 0, 0,
                                        0, 1e-10, 0, 0, 0, 0,
                                        0, 0, 1e-10, 0, 0, 0,
                                        0, 0, 0, 1e-10, 0, 0,
                                        0, 0, 0, 0, 1e-10, 0,
                                        0, 0, 0, 0, 0, 1e-10};
            VioCallbackOrientation.setRPY(gt_roll, pitch, gt_yaw);
            vio_odom_pose_msg_.pose.pose.orientation = tf2::toMsg(VioCallbackOrientation);
        
            vio_pub.publish(vio_odom_pose_msg_);
    };

    void markerArrayCallback(const aruco_msgs::MarkerArray::ConstPtr& markerArray) {
            
            geometry_msgs::TransformStamped inverseDroneToRealsenseTransform = tfBuffer.lookupTransform("realsense_frame","base_link", ros::Time(0));
            double smallest_z = std::numeric_limits<double>::max(); 
            const aruco_msgs::Marker* smallest_z_marker = nullptr;
            
            for (const auto& marker : markerArray->markers) {
                    if (marker.pose.pose.position.z < smallest_z) {
                        smallest_z = marker.pose.pose.position.z;
                        smallest_z_marker = &marker;
                    }
                }

            tf2::fromMsg(smallest_z_marker->pose.pose.orientation, q_marker_callback);
            tf2::Matrix3x3(q_marker_callback).getRPY(roll, pitch, yaw);
            q_marker_callback.normalize();
            geometry_msgs::TransformStamped mapToOdomTransform = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
            std::string markerFrameId = "marker_frame_" + std::to_string(smallest_z_marker->id);
            geometry_msgs::TransformStamped mapToMarkerTransform = tfBuffer.lookupTransform("map", markerFrameId, ros::Time(0));
            tf2::fromMsg(mapToMarkerTransform.transform.rotation, q_marker_callback);
            tf2::Matrix3x3(q_marker_callback).getRPY(roll_marker, pitch_marker, yaw_marker);
            q_marker_callback.normalize();
            geometry_msgs::TransformStamped markerToRealsenseTransform;
            markerToRealsenseTransform.transform.rotation = smallest_z_marker->pose.pose.orientation;
            
            tf2::fromMsg(markerToRealsenseTransform.transform.rotation, q_marker_callback);
            tf2::Matrix3x3(q_marker_callback).getRPY(roll, pitch, yaw);
            pitch_in_map_frame = roll + pitch_marker - M_PI;
            drone_map_x = -abs(sin(M_PI/2 - pitch_in_map_frame)*smallest_z_marker->pose.pose.position.z) + -abs(cos(M_PI/2 - pitch_in_map_frame)*smallest_z_marker->pose.pose.position.y) + mapToMarkerTransform.transform.translation.x;
            drone_map_z = cos(M_PI/2 - pitch_in_map_frame)*smallest_z_marker->pose.pose.position.z + sin(M_PI/2 - pitch_in_map_frame)*smallest_z_marker->pose.pose.position.y + mapToMarkerTransform.transform.translation.z;
            tf2::fromMsg(markerToRealsenseTransform.transform.rotation, q_marker_callback);
            tf2::Matrix3x3(q_marker_callback).getRPY(roll, pitch, yaw);
            markerPose.header.stamp = smallest_z_marker->header.stamp;
            markerPose.pose.pose.position.x = drone_map_x - mapToOdomTransform.transform.translation.x;
            markerPose.pose.pose.position.y = ground_truth_msg_.pose.position.y; 
            markerPose.pose.pose.position.z = drone_map_z - mapToOdomTransform.transform.translation.z;
            
            if (pitch_in_map_frame < -M_PI) {
                pitch_in_map_frame += 2*M_PI;
            }
                            
            newOrientation.setRPY(gt_roll, pitch_in_map_frame, gt_yaw);
            newOrientation.normalize();
            markerPose.pose.pose.orientation = tf2::toMsg(newOrientation);

            markerPose.pose.covariance = {1e-10, 0, 0, 0, 0, 0,
                                            0, 1e-10, 0, 0, 0, 0,
                                            0, 0, 1e-10, 0, 0, 0,
                                            0, 0, 0, 1e-10, 0, 0,
                                            0, 0, 0, 0, 1e-10, 0,
                                            0, 0, 0, 0, 0, 1e-10};
            tf2::doTransform(markerPose, markerPose, inverseDroneToRealsenseTransform);
            markerPose.header.frame_id = "odom";
            markerPose.header.stamp = ros::Time::now();
            marker_pub.publish(markerPose);
    
    };

    void EkfCallback(const nav_msgs::Odometry::ConstPtr& ekfOdom) {
        
        geometry_msgs::TransformStamped mapToOdomTransform = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
        if (reached_goal) {
            //kill the rosnode named ekf_se_map:
            std::system("rosnode kill ekf_se_map");
        }
    }

    void gazeboStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {   
        gazebo_msg = *msg;

        int mantisIndex = -1;
        for (size_t i = 0; i < gazebo_msg.name.size(); ++i)
        {
            if (gazebo_msg.name[i] == "mantis" || gazebo_msg.name[i] == "nxp_drone"|| gazebo_msg.name[i] == "agile")
            {
                mantisIndex = static_cast<int>(i);
                break;
            }
        }
        
        if (mantisIndex != -1 && mantisIndex < gazebo_msg.pose.size())
        {
            ground_truth_msg_.pose = gazebo_msg.pose[mantisIndex];
            ground_truth_msg_.header.stamp = ros::Time::now();
            ground_truth_msg_.header.frame_id = "map";

        tf2::fromMsg(ground_truth_msg_.pose.orientation, gt_q);
        tf2::Matrix3x3(gt_q).getRPY(gt_roll, gt_pitch, gt_yaw);
        
        //pitchCondition = checkPitchCondition(gt_pitch);
        //posCondition = checkPositionCondition({ground_truth_msg_.pose.position.x, ground_truth_msg_.pose.position.y, ground_truth_msg_.pose.position.z});
        
        //if (pitchCondition && posCondition) {
        //    reached_goal = true;
        //    ROS_INFO("marker pose estimator node reached goal.");
        //}

        }
    
    }
    

private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;
    ros::Subscriber markerSub;
    ros::Publisher vio_pub;
    ros::Publisher marker_pub;
    ros::Publisher dronePosePub;
    ros::Subscriber vio_odom_sub;
    ros::Timer publishTimer;
    ros::Subscriber ground_truth_sub_;
    ros::Subscriber ekf_sub;

    geometry_msgs::PoseWithCovarianceStamped markerPose;
    geometry_msgs::PoseStamped ground_truth_msg_;
    gazebo_msgs::ModelStates gazebo_msg;
    double prev_pos_x, prev_pos_z = 0;
    double curr_velocity_x, curr_velocity_z = 0;

    bool reached_goal = false;
    bool posCondition = false;
    bool pitchCondition = false;

    double first_vio = true;
    double roll, pitch, yaw;
    double pitch_in_map_frame;
    double roll_marker, pitch_marker, yaw_marker;
    double drone_map_x, drone_map_y, drone_map_z;
    double gt_roll, gt_pitch, gt_yaw;
    double estimated_pitch, estimated_roll, estimated_yaw;
    double ground_truth_pitch, ground_truth_roll, ground_truth_yaw;
    double twist_x_error, twist_z_error;
    double pos_x_error, pos_z_error;
    double prev_timestamp = 0;
    double curr_timestamp;
    double prev_pitch = 0;
    double pitch_vel = 0;
    double dt;

    geometry_msgs::Vector3 curr_velocity;

    tf2::Quaternion q_vio;
    tf2::Quaternion newOrientation;
    tf2::Quaternion q_vio_callback;
    tf2::Quaternion q_marker_callback;
    tf2::Quaternion gt_q;
        
    bool checkPositionCondition(std::vector<double> cur_pos) {
        
        double threshold_pos_x = 0.35;
        double threshold_pos_y = 1.5;
        double threshold_pos_z = 0.15;
        double platform_x = 0;
        double platform_y = 0;
        double platform_z = 1.25;

        return 
            (std::abs(cur_pos[0] - platform_x) < threshold_pos_x) &&
            (std::abs(cur_pos[1] - platform_y) < threshold_pos_y) &&
            (std::abs(cur_pos[2] - platform_z) < threshold_pos_z);
    }

    bool checkPitchCondition(double cur_pitch) {
        
    double platform_pitch = -0.44879895051;
    double threshold_pitch = 0.1;//0.3;

        return std::abs(cur_pitch - platform_pitch) < threshold_pitch;
    }
    
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "controller_node");

    ControllerNode node;
    ros::spin();
    return 0;
}
