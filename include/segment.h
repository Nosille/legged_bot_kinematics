#pragma once

#include <string>

/// @brief Class defining a segment for a legged robot
class Segment
{
  public:
    /// @brief Constructor for Segment
    /// @param _name = name of segment
    /// @param _length = length to parent segment
    /// @param _offset = offset of segment from straight when angle is zero
    /// @param _minAngle = min allowed angle of segment in radians
    /// @param _maxAngle = max allowed angle of segment in radians
    /// @param _maxRate = max allowed angular rate of segment in radians/sec  
    Segment(std::string _name, 
          float _length,
          float _offset,
          float _minAngle,
          float _maxAngle,
          float _maxRate,
          uint16_t _anglePrecisionFactor = 10000,
          uint16_t _lengthPrecisionFactor = 10000);

    Segment(std::string _name,
          const Segment& _oldSegment);

    /// @brief Get name of Segment
    /// @return name of segment
    std::string getName() const;

    /// @brief Get distance to next segment
    /// @return angle in radians
    float getLength() const;
    uint32_t getLengthAsInt() const;    

    /// @brief Get offset of segment from straight when angle is zero
    /// @return angle in radians
    float getAngleOffset() const;
    int32_t getAngleOffsetAsInt() const;

    /// @brief Get current angle of segment
    /// @return angle in radians
    float getAngleCurrent() const; 
    int32_t getAngleCurrentAsInt() const;   

    /// @brief Get min allowed angle of segment
    /// @return angle in radians
    float getAngleMin() const;
    int32_t getAngleMinAsInt() const;    

    /// @brief Get max allowed angle of segment
    /// @return angle in radians    
    float getAngleMax() const;
    int32_t getAngleMaxAsInt() const;
    
    /// @brief Get max allowed rate of segment
    /// @return angular rate in radians/sec    
    float getAngleRateMax() const;
    uint32_t getAngleRateMaxAsInt() const;

    /// @brief Get factor converting angular float values to an internal uint32_t
    /// @return multiplication factor going from float to uint32_t
    uint16_t getAnglePrecisionFactor() const;

    /// @brief Get factor converting linear float values to an internal uint32_t
    /// @return multiplication factor going from float to uint32_t
    uint16_t getLengthPrecisionFactor() const;    

    /// @brief Set distance to next segment
    /// @return false if fails to set
    bool setLength(float _length);

    /// @brief Set offset of segment from straight when angle is zero
    /// @return false if min > max
    bool setAngleOffset(float _angleOffset);

    /// @brief Set current angle of segment
    /// @return angular rate in radians/sec
    bool setAngleCurrent(float _angleCurrent);

    /// @brief Set min allowed angle of segment
    /// @return false if min > max
    bool setAngleMin(float _angleMin);

    /// @brief Set max allowed angle of segment
    /// @return false if max < min   
    bool setAngleMax(float _angleMax);

    /// @brief Set max allowed rate of segment
    /// @return angular rate in radians/sec
    bool setAngleRateMax(float _angleRateMax);

    /// @brief Set factor converting angular float values to an internal uint32_t 
    /// significantly improves processing on many microcontrollers
    bool setAnglePrecisionFactor(uint16_t _anglePrecisionFactor);

    /// @brief Set factor converting linear float values to an internal uint32_t 
    /// significantly improves processing on many microcontrollers
    bool setLengthPrecisionFactor(uint16_t _linearPrecisionFactor);    

  protected:
    
    // Global Variables
    std::string name_;
    uint32_t length_;
    int32_t angleOffset_;
    int32_t angleCurrent_;
    int32_t angleMin_;
    int32_t angleMax_;
    uint32_t angleRateMax_;

    uint16_t anglePrecisionFactor_;
    uint16_t lengthPrecisionFactor_;
};