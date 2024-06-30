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
    Bot(const std::string &_name, const std::vector<Leg> &_legs,
        const int _gait, const double _stepLength, const double _stepHeight);

    /// @brief Constructor for Bot using eigen vectors
    /// @param _name = name of bot
    /// @param _legIds = list of namse for each leg
    /// @param _origin = list of vectors for each leg origin relative to body center in meters
    /// @param _lengths = list of lengths of each leg member from origin to end in meters (coxa, femur, tibia, tarsus)
    /// @param _segmentOffsets offset of each segment from straight when encoder is at zero in radians
    /// @param _segmentMins minimum angle of each segment from encoder zero in radians
    /// @param _segmentMaxs maximum angle of each segment from encoder zero in radians
    /// @param _segmentRatess maximum angular rate of each segment in radians per second
    /// @param _gait gait
    /// @param _stepLength Length of each step in meters
    /// @param _stepHeight Height of each step in meters
    Bot(const std::string &_name, const std::list<std::string> &_legIds, const std::list<Eigen::Vector3d> &_legOrigins, 
        const std::list<Eigen::Vector4d> &_legLengths, const std::list<Eigen::Vector4d> &_segmentOffsets,
        const std::list<Eigen::Vector4d> &_segmentMins, const std::list<Eigen::Vector4d> &_segmentMaxs,
        const std::list<Eigen::Vector4d> &_segmentRates,
        const int _gait, const double _stepLength, const double _stepHeight);

    // Methods
    std::vector<double> setLegPosition(const std::string &_id, const Eigen::Vector3d &_point);
    std::vector<double> setLegPosition(int _index, const Eigen::Vector3d &_point);
    std::vector<std::vector<double>> setLegPositions(const std::vector<Eigen::Vector3d> &_positions);
    Eigen::Vector3d transform_pose(int _index, const Eigen::Vector3d &_pose, const Eigen::Vector3d &_translate, const Eigen::Quaterniond &_quaternion);
    

    /// @brief Get name of Bot
    /// @return name of bot
    std::string getName() const;
    std::vector<Leg> getLegs() const;

    int getGait() const;
    double getStepLength() const;
    double getStepHeight() const;

    bool setGait(int _gait);
    bool setStepLength(double _stepLength);
    bool setStepHeight(double _stepHeight);

  protected:
    double PI = atan(1)*4;
    
    // Global Variables
    std::string name_;
    std::string frame_id_;
    std::vector<Leg> legs_;
  
    int gait_;
    double stepLength_;
    double stepHeight_;

    // Methods
 
}; // class Bot

