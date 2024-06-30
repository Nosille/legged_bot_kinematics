#pragma once

#include <vector>
#include <map> 
#include <string>

#include <Eigen/Geometry> 

#include "segment.h"

/// @brief Class defining a leg for a multilegged robot
/// This model assumes an insect leg with any number of segments
/// the first segment is assumed to be a vertical pin.
/// All other segments are assumed to be a horizontal pin.
class Leg
{
  public:

    /// @brief Constructor for Leg using predefined segments
    /// @param _name = name of leg
    /// @param _origin = location of leg origin relative to body center in meters
    /// @param _segments = vector of segments (coxa, femur, tibia, tarsus)
    Leg(const std::string &_name, 
        const Eigen::Affine3f &_origin, 
        const std::vector<Segment> &_segments,
        uint16_t _anglePrecisionFactor = 10000,
        uint16_t _lengthPrecisionFactor = 10000);

    /// @brief Constructor for Leg using eigen vectors (limited to 4 segments)
    /// @param _name = name of leg
    /// @param _origin = location of leg origin (coxa segment) relative to body center in meters
    /// @param _segmentLengths = distance to next segment for each leg member from origin to end in meters(coxa-femur, femur-tibia, tibia-tarsus, tarsus-end)
    /// @param _segmentOffsets = offset of each segment from straight when encoder is at zero in radians
    /// @param _segmentMins = minimum angle of each segment from encoder zero in radians (values are < 0)
    /// @param _segmentMaxs  = maximum angle of each segment from encoder zero in radians (values are > 0)
    /// @param _segmentRates  = maximum angular rate of each segment in radians per second (values > 0)
    Leg(const std::string &_name,
        const Eigen::Affine3f &_origin,
        const Eigen::Vector4f &_segmentLengths,
        const Eigen::Vector4f &_segmentOffsets,
        const Eigen::Vector4f &_segmentMins,
        const Eigen::Vector4f &_segmentMaxs,
        const Eigen::Vector4f &_segmentRates,
        uint16_t _anglePrecisionFactor = 10000,
        uint16_t _lengthPrecisionFactor = 10000
       );

    /// @brief Constructor for Leg using eigen vectors (limited to 4 segments)
    /// @param _name = name of leg
    /// @param _origin = location of leg origin (coxa segment) relative to body center in meters
    /// @param _segmentLengths = distance to next segment for each leg member from origin to end in meters(coxa-femur, femur-tibia, tibia-tarsus, tarsus-end)
    /// @param _segmentOffsets = offset of each segment from straight when encoder is at zero in radians
    /// @param _segmentMins = minimum angle of each segment from encoder zero in radians (values are < 0)
    /// @param _segmentMaxs  = maximum angle of each segment from encoder zero in radians (values are > 0)
    /// @param _segmentRates  = maximum angular rate of each segment in radians per second (values > 0)
    Leg(const std::string &_name,
        const Eigen::VectorXf &_origin,
        const Eigen::Vector4f &_segmentLengths,
        const Eigen::Vector4f &_segmentOffsets,
        const Eigen::Vector4f &_segmentMins,
        const Eigen::Vector4f &_segmentMaxs,
        const Eigen::Vector4f &_segmentRates,
        uint16_t _anglePrecisionFactor = 10000,
        uint16_t _lengthPrecisionFactor = 10000
       );       

    /// @brief Constructor for Leg using float arrays
    /// @param _name = name of leg
    /// @param _origin = location of leg origin (coxa segment) relative to body center in meters (x, y, z, roll, pitch, yaw)
    /// @param _segmentLengths = distance to next segment for each leg member from origin to end in meters(coxa-femur, femur-tibia, tibia-tarsus, tarsus-end)
    /// @param _segmentOffsets = offset of each segment from straight when encoder is at zero in radians
    /// @param _segmentMins = minimum angle of each segment from encoder zero in radians
    /// @param _segmentMaxs  = maximum angle of each segment from encoder zero in radians
    /// @param _segmentRates  = maximum angular rate of each segment in radians per second    
    Leg(const std::string &_name,
        const float _origin[6],
        uint8_t count,
        const float *_segmentLengths,
        const float *_segmentOffsets,
        const float *_segmentMins,
        const float *_segmentMaxs,
        const float *_segmentRates,
        uint16_t _anglePrecisionFactor = 10000,
        uint16_t _lengthPrecisionFactor = 10000         
        );

    // Methods

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @param _angles return vector of ints containing segment angles in radians * precision factor (coxa, femur, tibia, tarsus) 
    void calcAnglesFromPoint(const Eigen::Vector<int32_t, 3> &_point, std::vector<int32_t> &_angles) const;    

       /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @param _angles return vector of floats containing segment angles in radians (coxa, femur, tibia, tarsus) 
    void calcAnglesFromPoint(const Eigen::Vector3f &_point, std::vector<float> &_angles) const;

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @return map containing angles for each named segment (coxa, femur, tibia, tarsus)
    std::map<std::string, float> calcAnglesFromPoint(const Eigen::Vector3f &_point) const;

    /// @brief Forward Kinematics
    /// @param _angles vector containing angles for each named segment (coxa, femur, tibia, tarsus)
    /// @param _point return vector containing x, y, z coordinates of foot in body coordinate system  
    void calcPointFromAngles(const std::vector<float> &_angles, Eigen::Vector3f &_point) const;

    /// @brief Forward Kinematics
    /// @param _angles vector containing angles for each named segment (coxa, femur, tibia, tarsus)
    /// @param _point return vector containing x, y, z coordinates of foot in body coordinate system  
    Eigen::Vector3f calcPointFromAngles(const Eigen::Vector4f &_angles) const;

    /// @brief Forward Kinematics
    /// @param _angles map containing angles for each named segment (coxa, femur, tibia, tarsus)
    /// @return vector containing x, y, z coordinates of foot in body coordinate system  
    Eigen::Vector3f calcPointFromAngles(const std::map<std::string, float> &_angles) const;
 
    /// @brief Get Current point from segment angles
    /// @return x, y, z coordinate of foot in body coordinate system
    const Eigen::Vector3f getPositionCurrent() const;

    /// @brief Get home position
    /// @return x, y, z coordinate of foot in body coordinate system
    const Eigen::Vector3f getPositionHome() const;    

    /// @brief Set Current foot position 
    /// @param _position x, y, z coordinates of foot in body coordinate system
    /// @return status
    bool setPositionCurrent(const Eigen::Vector3f &position);

    /// @brief Set home position
    /// @return x, y, z coordinate of foot in body coordinate system
    bool setPositionHome(const Eigen::Vector3f &position);   

    /// @brief Get distance to point
    /// @return x, y, z coordinates of the foot in a point centered coordinate system 
    Eigen::Vector3f calcDistance(const Eigen::Vector3f &_point);

    /// @brief Get distance to point
    /// @return the relative distance between the foot and a point
    float calcDistancexyz(const Eigen::Vector3f &_point);   

    /// @brief Get distance to point
    /// @return the the relative distance between the foot and a point in a plane parrallel to the body x/y plane
    float calcDistancexy(const Eigen::Vector3f &_point);   

    /// @brief Get distance to point
    /// @return the the relative distance between the foot and a point in a plane parrallel to the body x/z plane
    float calcDistancexz(const Eigen::Vector3f &_point);  

    /// @brief Get distance to point
    /// @return the the relative distance between the foot and a point in a plane parrallel to the body y/z plane
    float calcDistanceyz(const Eigen::Vector3f &_point);            

    /// @brief Get name of Leg
    /// @return name of leg
    std::string getName() const;

    /// @brief Get origin of leg in body coordinate system
    /// @return location of leg origin relative to body center in meters and radians
    Eigen::Affine3f getOrigin() const;

    /// @brief Get the portion of the step the leg is current in
    /// @return phase. 0 = stance (on ground), 1 = pre-swing, 2 = early swing (lifting), 3 = mid swing (travel), 4 = late swing (lowering), 5 = post-swing
    uint8_t getPhase() const;
    
    /// @brief Get segments
    /// @return segments from origin to end (coxa, femur, tibia, tarsus)
    std::vector<Segment> getSegments() const;

    /// @brief Get length to next segment
    /// @return vector of segment lengths
    void getSegmentLengths(Eigen::Vector4f &_lengths) const;

    /// @brief Get segment offsets
    /// @return vector of segment offsets
    void getSegmentAnglesOffset(Eigen::Vector4f &_angles) const;

    /// @brief Get segment angles
    /// @return vector of segment angles
    void getSegmentAnglesCurrent(Eigen::Vector4f &_angles) const;

    /// @brief Get segment min angles
    /// @return vector of min angles
    void getSegmentAnglesMin(Eigen::Vector4f &_angles) const;    

    /// @brief Get segment max angles
    /// @return vector of max angles
    void getSegmentAnglesMax(Eigen::Vector4f &_angles) const;  

    /// @brief Get segment offsets from straight at zero angle
    /// @return map of segment offsets
    std::map<std::string, float> getSegmentLengths() const;

    /// @brief Get segment offsets from straight at zero angle
    /// @return map of segment offsets
    std::map<std::string, float> getSegmentAnglesOffset() const;

    /// @brief Get segment angles
    /// @return map of segment angles
    std::map<std::string, float> getSegmentAnglesCurrent() const;

    /// @brief Get segment min angles
    /// @return map of segment min angles
    std::map<std::string, float> getSegmentAnglesMin() const;

    /// @brief Get segment angles
    /// @return map of segment max angles
    std::map<std::string, float> getSegmentAnglesMax() const;

    /// @brief Get factor converting angular float values to an internal uint32_t
    /// @return multiplication factor going from float to uint32_t
    uint16_t getAnglePrecisionFactor() const;

    /// @brief Get factor converting linear float values to an internal uint32_t
    /// @return multiplication factor going from float to uint32_t
    uint16_t getLengthPrecisionFactor() const;   

    /// @brief Set segment offsets from straight at zero angle
    /// @param _angles map of segment offsets
    /// @return false if failed to set any
    bool setSegmentLengths(const std::map<std::string, float> &_lengths);

    /// @brief Set segment offsets from straight at zero angle
    /// @param _anglesOffset map of segment offsets
    /// @return false if failed to set any
    bool setSegmentAnglesOffset(const std::map<std::string, float> &_anglesOffset);

    /// @brief Set segment angles
    /// @param _angles map of segment angles
    /// @return false if failed to set any
    bool setSegmentAnglesCurrent(const std::map<std::string, float> &_anglesCurrent);

    /// @brief Set segment min angles
    /// @param _angles map of segment min angles
    /// @return false if failed to set any
    bool setSegmentAnglesMin(const std::map<std::string, float> &_anglesMin);

    /// @brief Set segment max angles
    /// @param _angles map of segment max angles
    /// @return false if failed to set any
    bool setSegmentAnglesMax(const std::map<std::string, float> &_anglesMax);

    /// @brief Set factor converting angular float values to an internal uint32_t 
    /// significantly improves processing on many microcontrollers without FPU
    bool setAnglePrecisionFactor(uint16_t _anglePrecisionFactor);

    /// @brief Set factor converting linear float values to an internal uint32_t 
    /// significantly improves processing on many microcontrollers without FPU
    bool setLengthPrecisionFactor(uint16_t _linearPrecisionFactor);   

    /// @brief Calculate square root of a 32 bit integer
    /// @param n the integer to calculate the root of
    /// @return the resulting integer representing the square root
    static uint32_t isqrt32(uint32_t n);

    
  protected:

    // Global Variables
    std::string name_;
    Eigen::Affine3f origin_;
    Eigen::Affine3f originInv_;
    std::vector<Segment> segments_;

    uint32_t anglePrecisionFactor_;
    uint32_t lengthPrecisionFactor_;
    Eigen::Vector3f homePosition_;
    // Eigen::Vector3i homePositionInt_;
    uint8_t phase_;

}; // class Leg