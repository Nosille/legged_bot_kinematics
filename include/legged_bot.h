#pragma once

#include <string>
#include <vector>
#include <list>
#include <memory>

#include "leg.h"

/// @brief Class defining multilegged robot
class LeggedBot
{
  public:
    /// @brief Constructor for Bot using predefined legs
    /// @param _name = name of bot
    /// @param _legs = vector containing list of legs   
    LeggedBot(const std::string &_name, const std::vector<Leg> &_legs,
        const int _gait, const float _stepLength, const float _stepHeight);

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
    LeggedBot(const std::string &_name, const std::list<std::string> &_legIds, const std::list<Eigen::VectorXf> &_legOrigins, 
        const std::list<Eigen::Vector4f> &_legLengths, const std::list<Eigen::Vector4f> &_segmentOffsets,
        const std::list<Eigen::Vector4f> &_segmentMins, const std::list<Eigen::Vector4f> &_segmentMaxs,
        const std::list<Eigen::Vector4f> &_segmentRates,
        const int _gait, const float _stepLength, const float _stepHeight);

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
    LeggedBot(const std::string &_name, uint8_t legs, uint8_t segments, 
        const std::string *_legIds, const float *_legOrigins, 
        const float *_legLengths, const float *_segmentOffsets,
        const float *_segmentMins, const float *_segmentMaxs, const float *_segmentRates,
        const int _gait, const float _stepLength, const float _stepHeight);

    // Methods
    std::vector<float> calcLegPosition(const std::string &_id, const Eigen::Vector3f &_point);
    std::vector<float> calcLegPosition(uint8_t _index, const Eigen::Vector3f &_point);
    std::vector<std::vector<float>> calcLegPositions(const std::vector<Eigen::Vector3f> &_positions);
    Eigen::Vector3f transform_pose(uint8_t _index, const Eigen::Vector3f &_pose, const Eigen::Vector3f &_translate, const Eigen::Quaternionf &_quaternion);

    // Gets
    Eigen::Vector3f getLegPositionCurrent(uint8_t _index);
    Eigen::Vector3f getLegPositionHome(uint8_t _index);
    std::vector<Eigen::Vector3f> getLegPositionsCurrent();
    std::vector<Eigen::Vector3f> getLegPositionsHome();
    const Eigen::Affine3f getFootprintTransform() const;
    
    // Sets
    bool setLegPositionCurrent(uint8_t _index, const Eigen::Vector3f &_point);
    bool setLegPositionHome(uint8_t _index, const Eigen::Vector3f &_point);
    void setLegPositionsCurrent(const std::vector<Eigen::Vector3f> &_positions);
    void setLegPositionsHome(const std::vector<Eigen::Vector3f> &_positions);

    /// @brief Get name of Bot
    /// @return name of bot
    std::string getName() const;
    std::vector<Leg> getLegs() const;

    int getGait() const;
    float getStepLength() const;
    float getStepHeight() const;

    bool setGait(uint8_t _gait);
    bool setStepLength(float _stepLength);
    bool setStepHeight(float _stepHeight);

  protected:
   
    // Global Variables
    std::string name_;
    std::string frame_id_;
    std::vector<Leg> legs_;
  
    int gait_;
    float stepLength_;
    float stepHeight_;
    Eigen::Affine3f base_footprint_;

    // Methods
 
}; // class LeggedBot
