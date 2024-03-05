
#include <iterator>
#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "leg.h"

Leg::Leg(const std::string &_name, 
         const Eigen::Affine3d &_origin,
         const Eigen::Vector4d &_lengths,
         const Eigen::Vector4d &_offsets,         
         const std::vector<Joint> &_joints)
        : name_(_name)
{
  origin_  = _origin;
  lengths_ = _lengths;

  for(Joint joint : _joints)
  {
    joints_.push_back(joint);
  }
  
  while(joints_.size() < 4)
  {
    joints_.emplace_back(_name + std::to_string(joints_.size()), 0.0, -M_PI, M_PI, 5.23);
  }
}

Leg::Leg(const std::string &_name,
         const Eigen::Affine3d &_origin,
         const Eigen::Vector4d &_lengths,
         const Eigen::Vector4d &_jointOffsets,
         const Eigen::Vector4d &_jointMins,
         const Eigen::Vector4d &_jointMaxs,
         const Eigen::Vector4d &_jointRates
        )
        : name_(_name)
{
  origin_  = _origin;
  lengths_ = _lengths;

  joints_.emplace_back(_name + "_coxa",   _jointOffsets[0], _jointMins[0], _jointMaxs[0], _jointRates[0]);
  joints_.emplace_back(_name + "_femur",  _jointOffsets[1], _jointMins[1], _jointMaxs[1], _jointRates[1]);
  joints_.emplace_back(_name + "_tibia",  _jointOffsets[2], _jointMins[2], _jointMaxs[2], _jointRates[2]);
  joints_.emplace_back(_name + "_tarsus", _jointOffsets[3], _jointMins[3], _jointMaxs[3], _jointRates[3]);
}

Leg::Leg(const std::string &_name,
         const Eigen::Vector3d &_origin,
         const Eigen::Vector4d &_lengths,
         const Eigen::Vector4d &_jointOffsets,
         const Eigen::Vector4d &_jointMins,
         const Eigen::Vector4d &_jointMaxs,
         const Eigen::Vector4d &_jointRates         
        )
        : name_(_name)
{
  Eigen::Vector3d vector(_origin[0], _origin[1], _origin[2]);
  Eigen::Quaternion<double> q;
  q = Eigen::AngleAxis<double>(_jointOffsets[0], Eigen::Vector3d(0.0, 0.0, 1.0));
  origin_ = Eigen::Affine3d::Identity();
  origin_.translate(vector);
  origin_ .rotate(q);

  lengths_ = _lengths;

  joints_.emplace_back(_name + "_coxa",                0.0, _jointMins[0], _jointMaxs[0], _jointRates[0]);
  joints_.emplace_back(_name + "_femur",  _jointOffsets[1], _jointMins[1], _jointMaxs[1], _jointRates[1]);
  joints_.emplace_back(_name + "_tibia",  _jointOffsets[2], _jointMins[2], _jointMaxs[2], _jointRates[2]);
  joints_.emplace_back(_name + "_tarsus", _jointOffsets[3], _jointMins[3], _jointMaxs[3], _jointRates[3]);
}

void Leg::getAnglesFromPoint(const Eigen::Vector3d &_point, std::vector<double> &_angles)
{
  _angles.clear();

  // lengths of each link
  double coxa_length = lengths_[0];
  double femur_length = lengths_[1];
  double tibia_length = lengths_[2];
  double tarsus_length = lengths_[3];

  // offsets of each joint
  double coxa_offset = joints_[0].getOffset();
  double femur_offset = joints_[1].getOffset();
  double tibia_offset = joints_[2].getOffset();
  double tarsus_offset = joints_[3].getOffset();

  // Transform from body coordinate system to leg coordinate system
  Eigen::Vector3d point;
  point = origin_.inverse() * _point;

  // Coxa angle can be calculated at this point
  double coxa_angle = atan(point[1]/point[0]);
  double coxa_out = coxa_angle - coxa_offset;
  // while(coxa_out > +PI/2) coxa_out -= PI;
  // while(coxa_out < -PI/2) coxa_out += PI;
  
  _angles.push_back(coxa_out);
  Eigen::Quaternion<double> q;
  q = Eigen::AngleAxis<double>(-coxa_angle, Eigen::Vector3d(0.0, 0.0, 1.0));
  point = q * point;

  // Select tarsus slope
  double tarsus_slope = -PI/2;

  // Calculate position of tibia-tarsus joint 
  // assuming coxa is horizontal and tarsus is at selected slope
  double xr = std::abs(point[0]) - coxa_length - tarsus_length * cos(tarsus_slope);
  double yr = point[1];
  double zr = point[2] - tarsus_length * sin(tarsus_slope);

  // Form a triangle from the coxa-femur, femur-tibia, and tibia-tarsus joints
  // we can use the law of cosines to find its internal angles
 
  // length and slope of coxa-femur to tibia-tarsus edge
  double hyp = sqrt((xr*xr) + (zr*zr));
  double angle = atan(zr/xr);

  // femur joint
  double femur_numerator = (femur_length*femur_length) + (hyp*hyp) - (tibia_length*tibia_length);
  double femur_denominator = 2 * femur_length * hyp;
  double femur_angle = acos(femur_numerator / femur_denominator);
  femur_angle += angle;
  _angles.push_back(femur_angle - femur_offset);
  
  // tibia joint
  double tibia_numerator = (hyp*hyp) - (femur_length*femur_length) - (tibia_length*tibia_length);
  double tibia_denominator = 2 * femur_length * tibia_length;
  double tibia_angle = - acos(tibia_numerator / tibia_denominator);
  _angles.push_back(tibia_angle - tibia_offset);

  // tarsus joint
  double tarsus_angle = tarsus_slope - femur_angle - tibia_angle;
  _angles.push_back(tarsus_angle - tarsus_offset);
}

std::map<std::string, double> Leg::getAnglesFromPoint(const Eigen::Vector3d &_point)
{
  // Get angles
  std::vector<double> angles;
  getAnglesFromPoint(_point, angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];
  
  return map;
}

void Leg::getPointFromAngles(std::map<std::string,double> &_angles, Eigen::Vector3d &_point)
{
  // lengths of each link
  double coxa_length = lengths_[0];
  double femur_length = lengths_[1];
  double tibia_length = lengths_[2];
  double tarsus_length = lengths_[3];

  // offsets of each joint
  double coxa_offset = joints_[0].getOffset();
  double femur_offset = joints_[1].getOffset();
  double tibia_offset = joints_[2].getOffset();
  double tarsus_offset = joints_[3].getOffset();

  // Calculate location of foot in the plane of the leg (2D, yz plane(y horizontal, z up))
  // get angles of each link relative to y-axis (about x-axis)
  double coxa_angle = 0.0;
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    coxa_angle += coxa_index->second + coxa_offset; 

  double femur_slope  = 0.0;
  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    femur_slope += femur_index->second + femur_offset; 

  double tibia_slope  = femur_slope;
  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    tibia_slope += tibia_index->second + tibia_offset;  

  double tarsus_slope = tibia_slope; 
  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    tarsus_slope += tarsus_index->second + tarsus_offset;

  // Position of foot relative to connection to robot at coxa
  double xp = coxa_length + 
              femur_length*cos(femur_slope) + 
              tibia_length*cos(tibia_slope) +
              tarsus_length*cos(tarsus_slope);

  double yp = 0.0;

  double zp = 0.0 + 
              femur_length*sin(femur_slope) + 
              tibia_length*sin(tibia_slope) +
              tarsus_length*sin(tarsus_slope);
  
  // Create tranform from leg origin to end of leg
  Eigen::Vector3d vector(xp, yp, zp);

  Eigen::Quaternion<double> q;
  q = Eigen::AngleAxis<double>(coxa_angle, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Vector3d point = q * vector;

  // Shift point to body frame
  _point = origin_ * point;
}

/// @brief Get name of Leg
/// @return name of leg
std::string Leg::getName() const { return name_; }

/// @brief Get origin of leg in body coordinate system
/// @return location of leg origin relative to body center in meters and radians
Eigen::Affine3d Leg::getOrigin() const { return origin_; }

/// @brief Get joint distances used in kinematics
/// @return length in meters between joints from origin to end (coxa, femur, tibia, tarsus)
Eigen::Vector4d Leg::getLengths() const { return lengths_; }

/// @brief Get joints
/// @return joints from origin to end (coxa, femur, tibia, tarsus)
std::vector<Joint> Leg::getJoints() const { return joints_; }
