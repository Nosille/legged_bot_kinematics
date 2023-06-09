#pragma once


#include <vector>
#include <map> 
#include <string>
#include <iterator>
#include <algorithm>

#include <Eigen/Geometry> 

#include "joint.h"

/// @brief Class defining leg for multilegged robot
class Leg
{
  public:
    Leg(const std::string &_name,
        const Eigen::Affine3d &_origin,
        const Eigen::Vector4d &_lengths 
       );

    Leg(const std::string &_name,
        const Eigen::Vector4d &_origin,
        const Eigen::Vector4d &_lengths 
        );

    // Methods
    void getAnglesFromPoint(const Eigen::Vector3d &_point, std::vector<double> &_angles);
    void getPointFromAngles(std::map<std::string, double> &_angles, Eigen::Vector3d &_point);

    std::map<std::string, double> getAnglesFromPoint(const Eigen::Vector3d &_point);

    std::string getName() const;
    Eigen::Affine3d getOrigin() const;
    Eigen::Vector4d getLengths() const;

  protected:
    double PI = atan(1)*4;
    
    // Global Variables
    std::string name_;
    Eigen::Affine3d origin_;
    Eigen::Vector4d lengths_;

}; // class Leg
