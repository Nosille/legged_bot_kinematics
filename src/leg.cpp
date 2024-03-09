
#include <iterator>
#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "leg.h"

Leg::Leg(const std::string &_name, 
         const Eigen::Affine3d &_origin,
         const std::vector<Joint> &_joints)
        : name_(_name)
{
  origin_  = _origin;

  for(Joint joint : _joints)
  {
    joints_.push_back(joint);
  }
  
  while(joints_.size() < 4)
  {
    joints_.emplace_back(_name + std::to_string(joints_.size()), 0.0, 0.0, -M_PI, M_PI, 10.0);
  }
}

Leg::Leg(const std::string &_name,
         const Eigen::Affine3d &_origin,
         const Eigen::Vector4d &_jointLengths,
         const Eigen::Vector4d &_jointOffsets,
         const Eigen::Vector4d &_jointMins,
         const Eigen::Vector4d &_jointMaxs,
         const Eigen::Vector4d &_jointRates
        )
        : name_(_name)
{
  origin_  = _origin;

  joints_.emplace_back(_name + "_coxa",   _jointLengths[0], _jointOffsets[0], _jointMins[0], _jointMaxs[0], _jointRates[0]);
  joints_.emplace_back(_name + "_femur",  _jointLengths[1], _jointOffsets[1], _jointMins[1], _jointMaxs[1], _jointRates[1]);
  joints_.emplace_back(_name + "_tibia",  _jointLengths[2], _jointOffsets[2], _jointMins[2], _jointMaxs[2], _jointRates[2]);
  joints_.emplace_back(_name + "_tarsus", _jointLengths[3], _jointOffsets[3], _jointMins[3], _jointMaxs[3], _jointRates[3]);
}

Leg::Leg(const std::string &_name,
         const Eigen::Vector3d &_origin,
         const Eigen::Vector4d &_jointLengths,
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

  joints_.emplace_back(_name + "_coxa",   _jointLengths[0],              0.0, _jointMins[0], _jointMaxs[0], _jointRates[0]);
  joints_.emplace_back(_name + "_femur",  _jointLengths[1], _jointOffsets[1], _jointMins[1], _jointMaxs[1], _jointRates[1]);
  joints_.emplace_back(_name + "_tibia",  _jointLengths[2], _jointOffsets[2], _jointMins[2], _jointMaxs[2], _jointRates[2]);
  joints_.emplace_back(_name + "_tarsus", _jointLengths[3], _jointOffsets[3], _jointMins[3], _jointMaxs[3], _jointRates[3]);
}

void Leg::getAnglesFromPoint(const Eigen::Vector3d &_point, std::vector<double> &_angles) const
{
  _angles.clear();

  // lengths of each link
  double coxa_length  = joints_[0].getLength();
  double femur_length = joints_[1].getLength();
  double tibia_length = joints_[2].getLength();
  double tarsus_length= joints_[3].getLength();

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

std::map<std::string, double> Leg::getAnglesFromPoint(const Eigen::Vector3d &_point) const
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

void Leg::getPointFromAngles(const Eigen::Vector4d &_angles, Eigen::Vector3d &_point) const
{
  // lengths of each link
  double coxa_length  = joints_[0].getLength();
  double femur_length = joints_[1].getLength();
  double tibia_length = joints_[2].getLength();
  double tarsus_length= joints_[3].getLength();

  // offsets of each joint
  double coxa_offset = joints_[0].getOffset();
  double femur_offset = joints_[1].getOffset();
  double tibia_offset = joints_[2].getOffset();
  double tarsus_offset = joints_[3].getOffset();

  // Calculate location of foot in the plane of the leg (2D, yz plane(y horizontal, z up))
  // get angles of each link relative to y-axis (about x-axis)
  double coxa_angle = 0.0;
  coxa_angle += _angles[0] + coxa_offset; 

  double femur_slope  = 0.0;
  femur_slope += _angles[1] + femur_offset; 

  double tibia_slope  = femur_slope;
  tibia_slope += _angles[2] + tibia_offset;  

  double tarsus_slope = tibia_slope; 
  tarsus_slope += _angles[3] + tarsus_offset;

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

Eigen::Vector3d Leg::getPointFromAngles(const Eigen::Vector4d &_angles) const
{
  Eigen::Vector3d point;

  getPointFromAngles(_angles, point);
  
  return point;
}

Eigen::Vector3d Leg::getPointFromAngles(const std::map<std::string, double> &_angles) const
{
  Eigen::Vector3d point;
  
  // Extract angles
  double coxa_angle = 0.0;
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    coxa_angle += coxa_index->second; 

  double femur_angle  = 0.0;
  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    femur_angle += femur_index->second; 

  double tibia_angle  = 0.0;
  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    tibia_angle += tibia_index->second;  

  double tarsus_angle = 0.0; 
  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    tarsus_angle += tarsus_index->second;

  // Add results to vector
  Eigen::Vector4d angles;
  angles[0] = coxa_angle;
  angles[1] = femur_angle;
  angles[2] = tibia_angle;
  angles[3] = tarsus_angle;

  // Calculate point
  getPointFromAngles(angles, point);  

  return point; 
}

Eigen::Vector3d Leg::getCurrentPoint() const
{
  // Add results to map
  Eigen::Vector4d angles;
  angles[0] = joints_[0].getCurrentAngle();
  angles[1] = joints_[1].getCurrentAngle();
  angles[2] = joints_[2].getCurrentAngle();
  angles[3] = joints_[3].getCurrentAngle();

  return Leg::getPointFromAngles(angles);
}

Eigen::Vector3d Leg::getDistance(const Eigen::Vector3d &_point)
{
  Eigen::Vector3d distance;
  Eigen::Vector3d currentPoint = getCurrentPoint();

  distance[0] = currentPoint[0] - _point[0];
  distance[1] = currentPoint[1] - _point[1];
  distance[2] = currentPoint[2] - _point[2];

  return distance;
}

double Leg::getDistancexyz(const Eigen::Vector3d &_point)
{
  Eigen::Vector3d vector = getDistance(_point);

  double distance = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[1]*vector[1]);

  return distance;
}

double Leg::getDistancexy(const Eigen::Vector3d &_point)
{
  Eigen::Vector3d vector = getDistance(_point);

  double distance = sqrt(vector[0]*vector[0] + vector[1]*vector[1]);

  return distance;
}

double Leg::getDistancexz(const Eigen::Vector3d &_point)
{
  Eigen::Vector3d vector = getDistance(_point);

  double distance = sqrt(vector[0]*vector[0] + vector[2]*vector[2]);

  return distance;
}

double Leg::getDistanceyz(const Eigen::Vector3d &_point)
{
  Eigen::Vector3d vector = getDistance(_point);

  double distance = sqrt(vector[1]*vector[1] + vector[2]*vector[2]);

  return distance;
}

std::string Leg::getName() const { return name_; }

Eigen::Affine3d Leg::getOrigin() const { return origin_; }

std::vector<Joint> Leg::getJoints() const { return joints_; }

void Leg::getJointLengths(Eigen::Vector4d &_lengths) const
{
  // Add results to vector
  _lengths[0] = joints_[0].getLength();
  _lengths[1] = joints_[1].getLength();
  _lengths[2] = joints_[2].getLength();
  _lengths[3] = joints_[3].getLength();
}

void Leg::getJointOffsets(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = joints_[0].getOffset();
  _angles[1] = joints_[1].getOffset();
  _angles[2] = joints_[2].getOffset();
  _angles[3] = joints_[3].getOffset();
}

void Leg::getJointMinAngles(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = joints_[0].getMinAngle();
  _angles[1] = joints_[1].getMinAngle();
  _angles[2] = joints_[2].getMinAngle();
  _angles[3] = joints_[3].getMinAngle();
}

void Leg::getJointMaxAngles(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = joints_[0].getMaxAngle();
  _angles[1] = joints_[1].getMaxAngle();
  _angles[2] = joints_[2].getMaxAngle();
  _angles[3] = joints_[3].getMaxAngle();
}

void Leg::getJointCurrentAngles(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = joints_[0].getCurrentAngle();
  _angles[1] = joints_[1].getCurrentAngle();
  _angles[2] = joints_[2].getCurrentAngle();
  _angles[3] = joints_[3].getCurrentAngle();
}

std::map<std::string, double> Leg::getJointLengths() const
{
  // Get angles
  Eigen::Vector4d angles;
  getJointLengths(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getJointOffsets() const
{
  // Get angles
  Eigen::Vector4d angles;
  getJointOffsets(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getJointMinAngles() const
{
  // Get angles
  Eigen::Vector4d angles;
  getJointMinAngles(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getJointMaxAngles() const
{
  // Get angles
  Eigen::Vector4d angles;
  getJointMaxAngles(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getJointCurrentAngles() const
{
  // Get angles
  Eigen::Vector4d angles;
  getJointCurrentAngles(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

bool Leg::setJointLengths(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= joints_[0].setLength(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= joints_[1].setLength(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= joints_[2].setLength(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= joints_[3].setLength(tarsus_index->second);

  return result;
}

bool Leg::setJointOffsets(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= joints_[0].setOffset(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= joints_[1].setOffset(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= joints_[2].setOffset(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= joints_[3].setOffset(tarsus_index->second);

  return result;
}

bool Leg::setJointMinAngles(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= joints_[0].setMinAngle(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= joints_[1].setMinAngle(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= joints_[2].setMinAngle(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= joints_[3].setMinAngle(tarsus_index->second);

  return result;
}

bool Leg::setJointMaxAngles(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= joints_[0].setMaxAngle(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= joints_[1].setMaxAngle(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= joints_[2].setMaxAngle(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= joints_[3].setMaxAngle(tarsus_index->second);

  return result;
}
    
bool Leg::setJointCurrentAngles(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= joints_[0].setCurrentAngle(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= joints_[1].setCurrentAngle(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= joints_[2].setCurrentAngle(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= joints_[3].setCurrentAngle(tarsus_index->second);

  return result;
}
