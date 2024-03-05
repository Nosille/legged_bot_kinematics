#pragma once


#include <string>
#include <vector>
#include <list>
#include <memory>

#include "leg.h"

/// @brief Class defining multilegged robot
class Bot
{
  public:
    /// @brief Constructor for Bot using predefined legs
    /// @param _name = name of bot
    /// @param _legs = vector containing list of legs   
    Bot(const std::string &_name, const std::vector<Leg> &_legs);

    /// @brief Constructor for Bot using eigen vectors
    /// @param _name = name of bot
    /// @param _legIds = list of namse for each leg
    /// @param _origin = list of vectors for each leg origin relative to body center in meters
    /// @param _lengths = list of lengths of each leg member from origin to end in meters (coxa, femur, tibia, tarsus)
    /// @param _jointOffsets offset of each joint from straight when encoder is at zero in radians
    /// @param _jointMins minimum angle of each joint from encoder zero in radians
    /// @param _jointMaxs maximum angle of each joint from encoder zero in radians
    /// @param _jointRatess maximum angular rate of each joint in radians per second
    Bot(const std::string &_name, const std::list<std::string> &_legIds, const std::list<Eigen::Vector3d> &_legOrigins, 
        const std::list<Eigen::Vector4d> &_legLengths, const std::list<Eigen::Vector4d> &_jointOffsets,
        const std::list<Eigen::Vector4d> &_jointMins, const std::list<Eigen::Vector4d> &_jointMaxs,
        const std::list<Eigen::Vector4d> &_jointRates);

    // Methods
    const std::vector<Leg> getLegs();
    std::vector<double> setLegPosition(const std::string &_id, const Eigen::Vector3d &_point);
    std::vector<double> setLegPosition(int _index, const Eigen::Vector3d &_point);
    Eigen::Vector3d transform_pose(int _index, const Eigen::Vector3d &_pose, const Eigen::Vector3d &_translate, const Eigen::Quaterniond &_quaternion);
    

    /// @brief Get name of Bot
    /// @return name of bot
    std::string getName() const;

  protected:
    double PI = atan(1)*4;
    
    // Global Variables
    std::string name_;
    std::string frame_id_;
    std::vector<Leg> legs_;

    // Methods
 
}; // class Bot

