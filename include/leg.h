#pragma once


#include <vector>
#include <map> 
#include <string>

#include <Eigen/Geometry> 

#include "joint.h"

class Leg
{
  public:

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = location of leg origin relative to body center in meters
    /// @param _lengths = length of each leg member from origin to end in meters (coxa, femur, tibia, tarsus)
    Leg(const std::string &_name, 
        const Eigen::Affine3d &_origin, 
        const Eigen::Vector4d &_lengths,        
        const Eigen::Vector4d &_jointOffsets,        
        const std::vector<Joint> &_joints);

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = Transform from body center to leg origin in meters
    /// @param _lengths = length of each leg member from origin to end in meters (coxa, femur, tibia, tarsus)
    /// @param _jointOffsets = offset of each joint from straight when encoder is at zero in radians
    /// @param _jointMins = minimum angle of each joint from encoder zero in radians
    /// @param _jointMaxs  = maximum angle of each joint from encoder zero in radians
    /// @param _jointRates  = maximum angular rate of each joint in radians per second
    Leg(const std::string &_name,
        const Eigen::Affine3d &_origin,
        const Eigen::Vector4d &_lengths,
        const Eigen::Vector4d &_jointOffsets,
        const Eigen::Vector4d &_jointMins,
        const Eigen::Vector4d &_jointMaxs,
        const Eigen::Vector4d &_jointRates
       );

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = location of leg origin relative to body center in meters
    /// @param _lengths = length of each leg member from origin to end in meters(coxa, femur, tibia, tarsus)
    /// @param _jointOffsets = offset of each joint from straight when encoder is at zero in radians
    /// @param _jointMins = minimum angle of each joint from encoder zero in radians
    /// @param _jointMaxs  = maximum angle of each joint from encoder zero in radians
    /// @param _jointRates  = maximum angular rate of each joint in radians per second    
    Leg(const std::string &_name,
        const Eigen::Vector3d &_origin,
        const Eigen::Vector4d &_lengths,
        const Eigen::Vector4d &_offsets,
        const Eigen::Vector4d &_jointMins,
        const Eigen::Vector4d &_jointMaxs,
        const Eigen::Vector4d &_jointRates         
        );

    // Methods

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @param _angles return vector of doubles containing joint angles in radians (coxa, femur, tibia, tarsus) 
    void getAnglesFromPoint(const Eigen::Vector3d &_point, std::vector<double> &_angles);

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @return map containing angles for each named joint (coxa, femur, tibia, tarsus)
    std::map<std::string, double> getAnglesFromPoint(const Eigen::Vector3d &_point);


    /// @brief Forward Kinematics
    /// @param _angles map containing angles for each named joint (coxa, femur, tibia, tarsus)
    /// @param _point return vector containing x, y, z coordinates of foot in body coordinate system  
    void getPointFromAngles(std::map<std::string, double> &_angles, Eigen::Vector3d &_point);

    std::string getName() const;
    Eigen::Affine3d getOrigin() const;
    Eigen::Vector4d getLengths() const;
    std::vector<Joint> getJoints() const;

  protected:
    double PI = atan(1)*4;
    
    // Global Variables
    std::string name_;
    Eigen::Affine3d origin_;
    Eigen::Vector4d lengths_;
    std::vector<Joint> joints_;

}; // class Leg
