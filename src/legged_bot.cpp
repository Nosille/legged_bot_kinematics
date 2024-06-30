#include <iterator>
#include <algorithm>

#include "legged_bot.h"

LeggedBot::LeggedBot(const std::string &_name, const std::vector<Leg> &_legs,
        const int _gait, const float _stepLength, const float _stepHeight)
: name_(_name)
, base_footprint_(Eigen::Affine3f::Identity())
{
  gait_ = _gait;
  stepLength_ = _stepLength;
  stepHeight_ = _stepHeight;
  
  for(Leg leg : _legs)
  {
    legs_.push_back(leg);
  }
}

LeggedBot::LeggedBot(const std::string &_name, const std::list<std::string> &_legIds, const std::list<Eigen::VectorXf> &_legOrigins, 
         const std::list<Eigen::Vector4f> &_legLengths, const std::list<Eigen::Vector4f> &_segmentOffsets,
         const std::list<Eigen::Vector4f> &_segmentMins, const std::list<Eigen::Vector4f> &_segmentMaxs,
         const std::list<Eigen::Vector4f> &_segmentRates, 
         const int _gait, const float _stepLength, const float _stepHeight)
: name_(_name)
{
  assert(_legIds.size() == _legLengths.size());
  assert(_legIds.size() > 2);
  assert(_legIds.size() < 5);
  
  gait_ = _gait;
  stepLength_ = _stepLength;
  stepHeight_ = _stepHeight;

  auto it1 = _legOrigins.begin();
  auto it2 = _legLengths.begin();
  auto it3 = _segmentOffsets.begin();
  auto it4 = _segmentMins.begin();
  auto it5 = _segmentMaxs.begin();
  auto it6 = _segmentRates.begin();
  for(std::string legId : _legIds)
  {
    // Get origin
    Eigen::Vector3f origin = *it1;
    std::advance(it1, 1);
    
    // Get lengths
    Eigen::Vector4f lengths = *it2;
    std::advance(it2, 1);
    
    // Get offsets
    Eigen::Vector4f offsets = *it3;
    std::advance(it3, 1);

    // Get mins
    Eigen::Vector4f mins = *it4;
    std::advance(it4, 1);

    // Get maxs
    Eigen::Vector4f maxs = *it5;
    std::advance(it5, 1);

    // Get rates
    Eigen::Vector4f rates = *it6;
    std::advance(it6, 1);    

    // Create leg
    Leg leg(legId, origin, lengths, offsets, mins, maxs, rates);
    legs_.push_back(leg);
  }
}
    
LeggedBot::LeggedBot(const std::string &_name, uint8_t _legs, uint8_t _segments, 
         const std::string *_legIds, const float *_legOrigins, 
         const float *_segmentLengths, const float *_segmentOffsets,
         const float *_segmentMins, const float *_segmentMaxs, const float *_segmentRates, 
         const int _gait, const float _stepLength, const float _stepHeight)
: name_(_name)
{
  gait_ = _gait;
  stepLength_ = _stepLength;
  stepHeight_ = _stepHeight;

  for(int i = 0; i < _legs; i++)
  {
    // Get origin
    float origin[3] = {_legOrigins[i*3 + 0], _legOrigins[i*3 + 1], _legOrigins[i*3 + 2]};

    const std::string &legId = _legIds[i];
    const float* lengths = &_segmentLengths[i*_segments];
    const float* offsets = &_segmentOffsets[i*_segments];
    const float* mins = &_segmentMins[i*_segments];
    const float* maxs = &_segmentMaxs[i*_segments];
    const float* rates = &_segmentRates[i*_segments];

    // Create leg
    Leg leg(legId, origin, _segments, lengths, offsets, mins, maxs, rates);
    legs_.push_back(leg);
  }
}

std::vector<Leg> LeggedBot::getLegs() const
{
  return legs_;
}

std::vector<float> LeggedBot::calcLegPosition(uint8_t _index, const Eigen::Vector3f &_point)
{
  std::vector<float> angles;
  legs_[_index].calcAnglesFromPoint(_point, angles);
  return angles;
}

std::vector<float> LeggedBot::calcLegPosition(const std::string &_index, const Eigen::Vector3f &_point)
{
  std::vector<float> angles;
  std::shared_ptr<Leg> leg;
  for(Leg entry : legs_)
  {
    if(entry.getName() == _index)
    {
      leg = std::make_shared<Leg>(entry);
      break;
    }
  }
  
  if(leg) leg->calcAnglesFromPoint(_point, angles);
  return angles;
}

std::vector<std::vector<float>> LeggedBot::calcLegPositions(const std::vector<Eigen::Vector3f> &_positions)
{
  std::vector<std::vector<float>> angles;
  
  for(int i = 0; i < _positions.size(); i++)
  {
    angles[i] = calcLegPosition(i, _positions[i]);
  }

  return angles;
}

Eigen::Vector3f LeggedBot::getLegPositionCurrent(uint8_t _index)
{
  return legs_[_index].getPositionCurrent();
}

Eigen::Vector3f LeggedBot::getLegPositionHome(uint8_t _index)
{
  return legs_[_index].getPositionHome();
}

std::vector<Eigen::Vector3f> LeggedBot::getLegPositionsCurrent()
{
  std::vector<Eigen::Vector3f> positions;
  for(uint8_t i = 0; i < legs_.size(); i++)
  {
    positions.push_back(getLegPositionCurrent(i));
  }

  return positions;
}

std::vector<Eigen::Vector3f> LeggedBot::getLegPositionsHome()
{
  std::vector<Eigen::Vector3f> positions;
  for(uint8_t i = 0; i < legs_.size(); i++)
  {
    positions.push_back(getLegPositionHome(i));
  }

  return positions;
}

bool LeggedBot::setLegPositionCurrent(uint8_t _index, const Eigen::Vector3f &_point)
{
  return legs_[_index].setPositionCurrent(_point);
}

bool LeggedBot::setLegPositionHome(uint8_t _index, const Eigen::Vector3f &_point)
{
  return legs_[_index].setPositionHome(_point);
}

void LeggedBot::setLegPositionsCurrent(const std::vector<Eigen::Vector3f> &_positions)
{
  for(uint8_t i = 0; i < legs_.size(); i++)
  {
    setLegPositionCurrent(i, _positions[i]);
  }
}

void LeggedBot::setLegPositionsHome(const std::vector<Eigen::Vector3f> &_positions)
{
  for(uint8_t i = 0; i < legs_.size(); i++)
  {
    setLegPositionHome(i, _positions[i]);
  }
}

const Eigen::Affine3f LeggedBot::getFootprintTransform() const
{
  return base_footprint_;
}

Eigen::Vector3f LeggedBot::transform_pose(uint8_t _index, const Eigen::Vector3f &_pose, const Eigen::Vector3f &_translate, const Eigen::Quaternionf &_quaternion)
{
  Eigen::Affine3f transform(Eigen::Affine3f::Identity());
  transform.translation() = _translate;
  transform.linear() = _quaternion.toRotationMatrix();

  return transform * _pose;
}

std::string LeggedBot::getName() const { return name_; }

int LeggedBot::getGait() const { return gait_; }

float LeggedBot::getStepLength() const { return stepLength_; }

float LeggedBot::getStepHeight() const { return stepHeight_; }

bool LeggedBot::setGait(uint8_t _gait) 
{
  gait_ = _gait;

  return true;
}

bool LeggedBot::setStepLength(float _stepLength)
{
  stepLength_ = _stepLength;

  return true;
}

bool LeggedBot::setStepHeight(float _stepHeight)
{
  stepHeight_ = _stepHeight;

  return true;
}
