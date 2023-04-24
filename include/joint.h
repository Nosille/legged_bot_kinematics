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

class Joint
{
  public:
    Joint(const std::string &_name, 
          const double &_minAngle,
          const double &_maxAngle,
          const double &_maxRate);

    // Methods
    std::string getName() const;
    double getMinAngle() const;
    double getMaxAngle() const;
    double getMaxRate() const;

    void setMinAngle(double &_minAngle);
    void setMaxAngle(double &_maxAngle);
    void setMaxRate(double &_maxRate);

  protected:
    
    // Global Variables
    std::string name_;
    double minAngle_;
    double maxAngle_;
    double maxRate_;
};
