#pragma once

#include <string>
#include <iostream> 
#include <shared_mutex> 
#include <mutex> 
#include <thread> 


/// @brief Class defining a joint for a legged robot
class Joint
{
  public:
    /// @brief Constructor for Joint
    /// @param _name = name of joint
    /// @param _length = length to next joint
    /// @param _offset = offset of joint from straight when angle is zero
    /// @param _minAngle = min allowed angle of joint in radians
    /// @param _maxAngle = max allowed angle of joint in radians
    /// @param _maxRate = max allowed angular rate of joint in radians/sec  
    Joint(std::string _name, 
          double _length,
          double _offset,
          double _minAngle,
          double _maxAngle,
          double _maxRate);

    Joint(std::string _name,
          const Joint& _oldJoint);

    /// @brief Get name of Joint
    /// @return name of joint
    std::string getName() const;

    /// @brief Get distance to next joint
    /// @return angle in radians
    double getLength() const;

    /// @brief Get offset of joint from straight when angle is zero
    /// @return angle in radians
    double getOffset() const;

    /// @brief Get min allowed angle of joint
    /// @return angle in radians
    double getMinAngle() const;

    /// @brief Get max allowed angle of joint
    /// @return angle in radians    
    double getMaxAngle() const;
    
    /// @brief Get max allowed rate of joint
    /// @return angular rate in radians/sec    
    double getMaxRate() const;

    /// @brief Get current angle of joint
    /// @return angle in radians
    double getCurrentAngle() const;    

    /// @brief Set distance to next joint
    /// @return false if fails to set
    bool setLength(double _length);

    /// @brief Set offset of joint from straight when angle is zero
    /// @return false if min > max
    bool setOffset(double _offset);

    /// @brief Set min allowed angle of joint
    /// @return false if min > max
    bool setMinAngle(double _minAngle);

    /// @brief Set max allowed angle of joint
    /// @return false if max < min   
    bool setMaxAngle(double _maxAngle);

    /// @brief Set max allowed rate of joint
    /// @angular rate in radians/sec
    bool setMaxRate(double _maxRate);

    /// @brief Set current angle of joint
    /// @return angular rate in radians/sec
    bool setCurrentAngle(double _currentAngle);

  protected:
    
    // Global Variables
    std::string name_;
    double length_;
    double offset_;
    double minAngle_;
    double maxAngle_;
    double maxRate_;
    double currentAngle_;
};
