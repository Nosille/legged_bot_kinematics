#pragma once


#include <vector>
#include <list>
#include <map> 
#include <mutex>
#include <string>
#include <iterator>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include "joint.h"

class Leg
{
  public:
    Leg(const std::string &_name,
        const Eigen::Vector4d &_origin,
        const Eigen::Vector4d &_lengths 
       );
    
    // Methods
    void getAnglesFromPoint(const Eigen::Vector3d &_point, double (&_angles)[4]);
    void getPointFromAngles(std::map<std::string, double> &_angles, Eigen::Vector3d &_point);

    std::map<std::string, double> getAnglesFromPoint(const Eigen::Vector3d &_point);
    pybind11::dict getAnglesFromPointPy(const Eigen::Vector3d &_point);
    Eigen::Vector3d getPointFromAnglesPy(const pybind11::dict &_dict);

    std::string getName() const;
    Eigen::Vector4d getOrigin() const;
    Eigen::Vector4d getLengths() const;

  protected:
    double PI = atan(1)*4;
    
    // Global Variables
    std::string name_;
    Eigen::Vector4d origin_;
    Eigen::Vector4d lengths_;

}; // class Leg
