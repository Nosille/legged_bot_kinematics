#include "joint.h"

/// @brief Constructor for Joint
/// @param _name name of joint
/// @param _minAngle min allowed angle of joint in radians
/// @param _maxAngle max allowed angle of joint in radians
Joint::Joint(const std::string &_name, 
             const double &_minAngle = -1.5708,
             const double &_maxAngle = 1.5708,
             const double &_maxRate = 6.0  
        )
        : name_(_name)
{
  minAngle_ = _minAngle;
  maxAngle_ = _maxAngle;

  maxRate_ = _maxRate;
}

/// @brief Get name of Joint
/// @return name of jont
std::string Joint::getName() const { return name_; }

/// @brief Get min allowed angle of joint
/// @return angle in radians
double Joint::getMinAngle() const { return minAngle_; }

/// @brief Get max allowed angle of joint
/// @return angle in radians
double Joint::getMaxAngle() const { return maxAngle_; }

/// @brief Get max allowed rate of joint
/// @return angular rate in radians/sec
double Joint::getMaxRate() const { return maxRate_; }

/// @brief Set min allowed angle of joint
/// @return angle in radians
void Joint::setMinAngle(double &_minAngle) { minAngle_ = _minAngle; }

/// @brief Set max allowed angle of joint
/// @return angle in radians
void Joint::setMaxAngle(double &_maxAngle) { maxAngle_ = _maxAngle; }

/// @brief Set max allowed rate of joint
/// @return angular rate in radians/sec
void Joint::setMaxRate(double &_maxRate) { maxRate_ = _maxRate; }
