#pragma once


#include <vector>

#include <string>
#include <iterator>
#include <algorithm>

#include <Eigen/Geometry> 

/// @brief Class defining joint for legged robot
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
