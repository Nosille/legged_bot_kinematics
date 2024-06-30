#pragma once


#include <vector>
#include <map> 
#include <string>

#include <Eigen/Geometry> 

#include "segment.h"

class Leg
{
  public:

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = location of leg origin relative to body center in meters
    /// @param _segments = vector of segments (coxa, femur, tibia, tarsus)
    Leg(const std::string &_name, 
        const Eigen::Affine3d &_origin, 
        const std::vector<Segment> &_segments);

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = location of leg origin (coxa segment) relative to body center in meters
    /// @param _segmentLengths = distance to next segment for each leg member from origin to end in meters(coxa-femur, femur-tibia, tibia-tarsus, tarsus-end)
    /// @param _segmentOffsets = offset of each segment from straight when encoder is at zero in radians
    /// @param _segmentMins = minimum angle of each segment from encoder zero in radians
    /// @param _segmentMaxs  = maximum angle of each segment from encoder zero in radians
    /// @param _segmentRates  = maximum angular rate of each segment in radians per second
    Leg(const std::string &_name,
        const Eigen::Affine3d &_origin,
        const Eigen::Vector4d &_segmentLengths,
        const Eigen::Vector4d &_segmentOffsets,
        const Eigen::Vector4d &_segmentMins,
        const Eigen::Vector4d &_segmentMaxs,
        const Eigen::Vector4d &_segmentRates
       );

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = location of leg origin (coxa segment) relative to body center in meters
    /// @param _segmentLengths = distance to next segment for each leg member from origin to end in meters(coxa-femur, femur-tibia, tibia-tarsus, tarsus-end)
    /// @param _segmentOffsets = offset of each segment from straight when encoder is at zero in radians
    /// @param _segmentMins = minimum angle of each segment from encoder zero in radians
    /// @param _segmentMaxs  = maximum angle of each segment from encoder zero in radians
    /// @param _segmentRates  = maximum angular rate of each segment in radians per second    
    Leg(const std::string &_name,
        const Eigen::Vector3d &_origin,
        const Eigen::Vector4d &_segmentLengths,
        const Eigen::Vector4d &_segmentOffsets,
        const Eigen::Vector4d &_segmentMins,
        const Eigen::Vector4d &_segmentMaxs,
        const Eigen::Vector4d &_segmentRates         
        );

    // Methods

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @param _angles return vector of doubles containing segment angles in radians (coxa, femur, tibia, tarsus) 
    void getAnglesFromPoint(const Eigen::Vector3d &_point, std::vector<double> &_angles) const;

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @return map containing angles for each named segment (coxa, femur, tibia, tarsus)
    std::map<std::string, double> getAnglesFromPoint(const Eigen::Vector3d &_point) const;

    /// @brief Forward Kinematics
    /// @param _angles vector containing angles for each named segment (coxa, femur, tibia, tarsus)
    /// @param _point return vector containing x, y, z coordinates of foot in body coordinate system  
    void getPointFromAngles(const Eigen::Vector4d &_angles, Eigen::Vector3d &_point) const;

    /// @brief Forward Kinematics
    /// @param _angles vector containing angles for each named segment (coxa, femur, tibia, tarsus)
    /// @param _point return vector containing x, y, z coordinates of foot in body coordinate system  
    Eigen::Vector3d getPointFromAngles(const Eigen::Vector4d &_angles) const;

    /// @brief Forward Kinematics
    /// @param _angles map containing angles for each named segment (coxa, femur, tibia, tarsus)
    /// @return vector containing x, y, z coordinates of foot in body coordinate system  
    Eigen::Vector3d getPointFromAngles(const std::map<std::string, double> &_angles) const;
 
    /// @brief Get Current point for segment angles
    /// @return x, y, z coordinate of foot in body coordinate system
    Eigen::Vector3d getCurrentPoint() const;

    /// @brief Get distance to point
    /// @return x, y, z coordinates of the foot in a point centered coordinate system 
    Eigen::Vector3d getDistance(const Eigen::Vector3d &_point);

    /// @brief Get distance to point
    /// @return the relative distance between the foot and a point
    double getDistancexyz(const Eigen::Vector3d &_point);   

    /// @brief Get distance to point
    /// @return the the relative distance between the foot and a point in a plane parrallel to the body x/y plane
    double getDistancexy(const Eigen::Vector3d &_point);   

    /// @brief Get distance to point
    /// @return the the relative distance between the foot and a point in a plane parrallel to the body x/z plane
    double getDistancexz(const Eigen::Vector3d &_point);  

    /// @brief Get distance to point
    /// @return the the relative distance between the foot and a point in a plane parrallel to the body y/z plane
    double getDistanceyz(const Eigen::Vector3d &_point);            

    /// @brief Get name of Leg
    /// @return name of leg
    std::string getName() const;

    /// @brief Get origin of leg in body coordinate system
    /// @return location of leg origin relative to body center in meters and radians
    Eigen::Affine3d getOrigin() const;
    
    /// @brief Get segments
    /// @return segments from origin to end (coxa, femur, tibia, tarsus)
    std::vector<Segment> getSegments() const;

    /// @brief Get length to next segment
    /// @return vector of segment lengths
    void getSegmentLengths(Eigen::Vector4d &_lengths) const;

    /// @brief Get segment offsets
    /// @return vector of segment offsets
    void getSegmentOffsets(Eigen::Vector4d &_angles) const;

    /// @brief Get segment min angles
    /// @return vector of min angles
    void getSegmentMinAngles(Eigen::Vector4d &_angles) const;    

    /// @brief Get segment max angles
    /// @return vector of max angles
    void getSegmentMaxAngles(Eigen::Vector4d &_angles) const;  

    /// @brief Get segment angles
    /// @return vector of segment angles
    void getSegmentCurrentAngles(Eigen::Vector4d &_angles) const;

    /// @brief Get segment offsets from straight at zero angle
    /// @return map of segment offsets
    std::map<std::string, double> getSegmentLengths() const;

    /// @brief Get segment offsets from straight at zero angle
    /// @return map of segment offsets
    std::map<std::string, double> getSegmentOffsets() const;

    /// @brief Get segment min angles
    /// @return map of segment min angles
    std::map<std::string, double> getSegmentMinAngles() const;

    /// @brief Get segment angles
    /// @return map of segment max angles
    std::map<std::string, double> getSegmentMaxAngles() const;

    /// @brief Get segment angles
    /// @return map of segment angles
    std::map<std::string, double> getSegmentCurrentAngles() const;

    /// @brief Set segment offsets from straight at zero angle
    /// @param _angles map of segment offsets
    /// @return false if failed to set any
    bool setSegmentLengths(const std::map<std::string, double> &_lengths);

    /// @brief Set segment offsets from straight at zero angle
    /// @param _angles map of segment offsets
    /// @return false if failed to set any
    bool setSegmentOffsets(const std::map<std::string, double> &_offsets);

        /// @brief Set segment min angles
    /// @param _angles map of segment min angles
    /// @return false if failed to set any
    bool setSegmentMinAngles(const std::map<std::string, double> &_minAngles);

    /// @brief Set segment max angles
    /// @param _angles map of segment max angles
    /// @return false if failed to set any
    bool setSegmentMaxAngles(const std::map<std::string, double> &_maxAngles);

    /// @brief Set segment angles
    /// @param _angles map of segment angles
    /// @return false if failed to set any
    bool setSegmentCurrentAngles(const std::map<std::string, double> &_currentAngles);


  protected:
    double PI = atan(1)*4;
    
    // Global Variables
    std::string name_;
    Eigen::Affine3d origin_;
    std::vector<Segment> segments_;

}; // class Leg
