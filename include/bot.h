#pragma once


#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include "leg.h"

/// @brief Class defining multilegged robot
class Bot
{
  public:
    // Bot(const std::string &_name, const std::string &_frameId, const std::list<std::string> &_legIds, const double wait_for_tf_delay);
    Bot(const std::string &_name, const std::list<Eigen::Vector4d> &_origin, const std::list<std::string> &_legIds, const std::list<double> &_legLengths);

    // Methods
    std::vector<double> setLegPosition(const std::string &_id, const Eigen::Vector3d &_point);
    std::vector<double> setLegPosition(int _index, const Eigen::Vector3d &_point);
    
    std::string getName() const;

  protected:
    double PI = atan(1)*4;
    
    // ROS 
    // ros::NodeHandle nh_;

    // std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    // double wait_for_tf_delay_;

    // Global Variables
    std::string name_;
    std::string frame_id_;
    std::vector<Leg> legs_;

    // Methods
    // bool getTransform(geometry_msgs::TransformStamped &_transform, const std::string &_target_frame, const std::string &_source_frame, const ros::Time &_time, const ros::Duration &_wait_for_tf_delay);
  
}; // class Bot

