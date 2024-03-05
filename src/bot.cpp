#include <iterator>
#include <algorithm>
#include <Eigen/Geometry> 

#include "bot.h"

Bot::Bot(const std::string &_name, const std::vector<Leg> &_legs)
: name_(_name)
{
  for(Leg leg : _legs)
  {
    legs_.push_back(leg);
  }
}
    
Bot::Bot(const std::string &_name, const std::list<std::string> &_legIds, const std::list<Eigen::Vector3d> &_legOrigins, 
         const std::list<Eigen::Vector4d> &_legLengths, const std::list<Eigen::Vector4d> &_jointOffsets,
         const std::list<Eigen::Vector4d> &_jointMins, const std::list<Eigen::Vector4d> &_jointMaxs,
         const std::list<Eigen::Vector4d> &_jointRates)
: name_(_name)
{
  assert(_legIds.size() == _legLengths.size());
  assert(_legIds.size() > 2);
  assert(_legIds.size() < 5);
  
  auto it1 = _legOrigins.begin();
  auto it2 = _legLengths.begin();
  auto it3 = _jointOffsets.begin();
  auto it4 = _jointMins.begin();
  auto it5 = _jointMaxs.begin();
  auto it6 = _jointRates.begin();
  for(std::string legId : _legIds)
  {
    // Get origin
    Eigen::Vector3d origin = *it1;
    std::advance(it1, 1);
    
    // Get lengths
    Eigen::Vector4d lengths = *it2;
    std::advance(it2, 1);
    
    // Get offsets
    Eigen::Vector4d offsets = *it3;
    std::advance(it3, 1);

    // Get mins
    Eigen::Vector4d mins = *it4;
    std::advance(it4, 1);

    // Get maxs
    Eigen::Vector4d maxs = *it5;
    std::advance(it5, 1);

    // Get rates
    Eigen::Vector4d rates = *it6;
    std::advance(it6, 1);    

    // Create leg
    Leg leg(legId, origin, lengths, offsets, mins, maxs, rates);
    legs_.push_back(leg);
  }
}

const std::vector<Leg> Bot::getLegs()
{
  return legs_;
}

std::vector<double> Bot::setLegPosition(int _index, const Eigen::Vector3d &_point)
{
  std::vector<double> angles;
  Eigen::Affine3d origin = legs_[_index].getOrigin();
  Eigen::Vector4d lengths = legs_[_index].getLengths();
  legs_[_index].getAnglesFromPoint(_point, angles);
  return angles;
}

std::vector<double> Bot::setLegPosition(const std::string &_index, const Eigen::Vector3d &_point)
{
  std::vector<double> angles;
  std::shared_ptr<Leg> leg;
  for(Leg entry : legs_)
  {
    if(entry.getName() == _index)
    {
      leg = std::make_shared<Leg>(entry);
      break;
    }
  }
  
  if(leg) leg->getAnglesFromPoint(_point, angles);
  return angles;
}

Eigen::Vector3d Bot::transform_pose(int _index, const Eigen::Vector3d &_pose, const Eigen::Vector3d &_translate, const Eigen::Quaterniond &_quaternion)
{
  Eigen::Affine3d transform(Eigen::Affine3d::Identity());
  transform.translation() = _translate;
  transform.linear() = _quaternion.toRotationMatrix();

  return transform * _pose;
}

std::string Bot::getName() const { return name_; }
