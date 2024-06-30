#pragma once

#include <string>
#include <iostream> 
#include <shared_mutex> 
#include <mutex> 
#include <thread> 


/// @brief Class defining a Segment for a legged robot
class Segment
{
  public:
    /// @brief Constructor for Segment
    /// @param _name = name of Segment
    /// @param _length = length to parent Segment
    /// @param _offset = offset of Segment from straight when angle is zero
    /// @param _minAngle = min allowed angle of Segment in radians
    /// @param _maxAngle = max allowed angle of Segment in radians
    /// @param _maxRate = max allowed angular rate of Segment in radians/sec  
    Segment(std::string _name, 
          double _length,
          double _offset,
          double _minAngle,
          double _maxAngle,
          double _maxRate);

    Segment(std::string _name,
          const Segment& _oldSegment);

    /// @brief Get name of Segment
    /// @return name of Segment
    std::string getName() const;

    /// @brief Get distance to next Segment
    /// @return angle in radians
    double getLength() const;

    /// @brief Get offset of Segment from straight when angle is zero
    /// @return angle in radians
    double getOffset() const;

    /// @brief Get min allowed angle of Segment
    /// @return angle in radians
    double getMinAngle() const;

    /// @brief Get max allowed angle of Segment
    /// @return angle in radians    
    double getMaxAngle() const;
    
    /// @brief Get max allowed rate of Segment
    /// @return angular rate in radians/sec    
    double getMaxRate() const;

    /// @brief Get current angle of Segment
    /// @return angle in radians
    double getCurrentAngle() const;    

    /// @brief Set distance to next Segment
    /// @return false if fails to set
    bool setLength(double _length);

    /// @brief Set offset of Segment from straight when angle is zero
    /// @return false if min > max
    bool setOffset(double _offset);

    /// @brief Set min allowed angle of Segment
    /// @return false if min > max
    bool setMinAngle(double _minAngle);

    /// @brief Set max allowed angle of Segment
    /// @return false if max < min   
    bool setMaxAngle(double _maxAngle);

    /// @brief Set max allowed rate of Segment
    /// @angular rate in radians/sec
    bool setMaxRate(double _maxRate);

    /// @brief Set current angle of Segment
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
