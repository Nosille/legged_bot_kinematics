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
    /// @param _joints = vector of joints (coxa, femur, tibia, tarsus)
    Leg(const std::string &_name, 
        const Eigen::Affine3d &_origin, 
        const std::vector<Joint> &_joints);

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = location of leg origin (coxa joint) relative to body center in meters
    /// @param _jointLengths = distance to next joint for each leg member from origin to end in meters(coxa-femur, femur-tibia, tibia-tarsus, tarsus-end)
    /// @param _jointOffsets = offset of each joint from straight when encoder is at zero in radians
    /// @param _jointMins = minimum angle of each joint from encoder zero in radians
    /// @param _jointMaxs  = maximum angle of each joint from encoder zero in radians
    /// @param _jointRates  = maximum angular rate of each joint in radians per second
    Leg(const std::string &_name,
        const Eigen::Affine3d &_origin,
        const Eigen::Vector4d &_jointLengths,
        const Eigen::Vector4d &_jointOffsets,
        const Eigen::Vector4d &_jointMins,
        const Eigen::Vector4d &_jointMaxs,
        const Eigen::Vector4d &_jointRates
       );

    /// @brief Constructor for Leg using eigen vectors
    /// @param _name = name of leg
    /// @param _origin = location of leg origin (coxa joint) relative to body center in meters
    /// @param _jointLengths = distance to next joint for each leg member from origin to end in meters(coxa-femur, femur-tibia, tibia-tarsus, tarsus-end)
    /// @param _jointOffsets = offset of each joint from straight when encoder is at zero in radians
    /// @param _jointMins = minimum angle of each joint from encoder zero in radians
    /// @param _jointMaxs  = maximum angle of each joint from encoder zero in radians
    /// @param _jointRates  = maximum angular rate of each joint in radians per second    
    Leg(const std::string &_name,
        const Eigen::Vector3d &_origin,
        const Eigen::Vector4d &_jointLengths,
        const Eigen::Vector4d &_jointOffsets,
        const Eigen::Vector4d &_jointMins,
        const Eigen::Vector4d &_jointMaxs,
        const Eigen::Vector4d &_jointRates         
        );

    // Methods

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @param _angles return vector of doubles containing joint angles in radians (coxa, femur, tibia, tarsus) 
    void getAnglesFromPoint(const Eigen::Vector3d &_point, std::vector<double> &_angles) const;

    /// @brief Inverse Kinematics
    /// @param _point x, y, z coordinates of foot in body coordinate system 
    /// @return map containing angles for each named joint (coxa, femur, tibia, tarsus)
    std::map<std::string, double> getAnglesFromPoint(const Eigen::Vector3d &_point) const;

    /// @brief Forward Kinematics
    /// @param _angles vector containing angles for each named joint (coxa, femur, tibia, tarsus)
    /// @param _point return vector containing x, y, z coordinates of foot in body coordinate system  
    void getPointFromAngles(const Eigen::Vector4d &_angles, Eigen::Vector3d &_point) const;

    /// @brief Forward Kinematics
    /// @param _angles vector containing angles for each named joint (coxa, femur, tibia, tarsus)
    /// @param _point return vector containing x, y, z coordinates of foot in body coordinate system  
    Eigen::Vector3d getPointFromAngles(const Eigen::Vector4d &_angles) const;

    /// @brief Forward Kinematics
    /// @param _angles map containing angles for each named joint (coxa, femur, tibia, tarsus)
    /// @return vector containing x, y, z coordinates of foot in body coordinate system  
    Eigen::Vector3d getPointFromAngles(const std::map<std::string, double> &_angles) const;
 
    /// @brief Get Current point for joint angles
    /// @return x, y, z coordinate of foot in body coordinate system
    Eigen::Vector3d getCurrentPoint() const;

    /// @brief Get distance to point
    /// @return x, y, z coordinate of foot in body coordinate system
    Eigen::Vector3d getDistance(const Eigen::Vector3d &_point);

    /// @brief Get name of Leg
    /// @return name of leg
    std::string getName() const;

    /// @brief Get origin of leg in body coordinate system
    /// @return location of leg origin relative to body center in meters and radians
    Eigen::Affine3d getOrigin() const;
    
    /// @brief Get joints
    /// @return joints from origin to end (coxa, femur, tibia, tarsus)
    std::vector<Joint> getJoints() const;

    /// @brief Get length to next joint
    /// @return vector of joint lengths
    void getJointLengths(Eigen::Vector4d &_lengths) const;

    /// @brief Get joint offsets
    /// @return vector of joint offsets
    void getJointOffsets(Eigen::Vector4d &_angles) const;

    /// @brief Get joint min angles
    /// @return vector of min angles
    void getJointMinAngles(Eigen::Vector4d &_angles) const;    

    /// @brief Get joint max angles
    /// @return vector of max angles
    void getJointMaxAngles(Eigen::Vector4d &_angles) const;  

    /// @brief Get joint angles
    /// @return vector of joint angles
    void getJointCurrentAngles(Eigen::Vector4d &_angles) const;

    /// @brief Get joint offsets from straight at zero angle
    /// @return map of joint offsets
    std::map<std::string, double> getJointLengths() const;

    /// @brief Get joint offsets from straight at zero angle
    /// @return map of joint offsets
    std::map<std::string, double> getJointOffsets() const;

    /// @brief Get joint min angles
    /// @return map of joint min angles
    std::map<std::string, double> getJointMinAngles() const;

    /// @brief Get joint angles
    /// @return map of joint max angles
    std::map<std::string, double> getJointMaxAngles() const;

    /// @brief Get joint angles
    /// @return map of joint angles
    std::map<std::string, double> getJointCurrentAngles() const;

    /// @brief Set joint offsets from straight at zero angle
    /// @param _angles map of joint offsets
    /// @return false if failed to set any
    bool setJointLengths(const std::map<std::string, double> &_lengths);

    /// @brief Set joint offsets from straight at zero angle
    /// @param _angles map of joint offsets
    /// @return false if failed to set any
    bool setJointOffsets(const std::map<std::string, double> &_offsets);

        /// @brief Set joint min angles
    /// @param _angles map of joint min angles
    /// @return false if failed to set any
    bool setJointMinAngles(const std::map<std::string, double> &_minAngles);

    /// @brief Set joint max angles
    /// @param _angles map of joint max angles
    /// @return false if failed to set any
    bool setJointMaxAngles(const std::map<std::string, double> &_maxAngles);

    /// @brief Set joint angles
    /// @param _angles map of joint angles
    /// @return false if failed to set any
    bool setJointCurrentAngles(const std::map<std::string, double> &_currentAngles);


  protected:
    double PI = atan(1)*4;
    
    // Global Variables
    std::string name_;
    Eigen::Affine3d origin_;
    std::vector<Joint> joints_;

}; // class Leg
