
#include <iterator>
#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "leg.h"

Leg::Leg(const std::string &_name, 
         const Eigen::Affine3d &_origin,
         const std::vector<Segment> &_segments)
        : name_(_name)
{
  origin_  = _origin;

  for(Segment segment : _segments)
  {
    segments_.push_back(segment);
  }
  
  while(segments_.size() < 4)
  {
    segments_.emplace_back(_name + std::to_string(segments_.size()), 0.0, 0.0, -M_PI, M_PI, 10.0);
  }
}

Leg::Leg(const std::string &_name,
         const Eigen::Affine3d &_origin,
         const Eigen::Vector4d &_segmentLengths,
         const Eigen::Vector4d &_segmentOffsets,
         const Eigen::Vector4d &_segmentMins,
         const Eigen::Vector4d &_segmentMaxs,
         const Eigen::Vector4d &_segmentRates
        )
        : name_(_name)
{
  origin_  = _origin;

  segments_.emplace_back(_name + "_coxa",   _segmentLengths[0], _segmentOffsets[0], _segmentMins[0], _segmentMaxs[0], _segmentRates[0]);
  segments_.emplace_back(_name + "_femur",  _segmentLengths[1], _segmentOffsets[1], _segmentMins[1], _segmentMaxs[1], _segmentRates[1]);
  segments_.emplace_back(_name + "_tibia",  _segmentLengths[2], _segmentOffsets[2], _segmentMins[2], _segmentMaxs[2], _segmentRates[2]);
  segments_.emplace_back(_name + "_tarsus", _segmentLengths[3], _segmentOffsets[3], _segmentMins[3], _segmentMaxs[3], _segmentRates[3]);
}

Leg::Leg(const std::string &_name,
         const Eigen::Vector3d &_origin,
         const Eigen::Vector4d &_segmentLengths,
         const Eigen::Vector4d &_segmentOffsets,
         const Eigen::Vector4d &_segmentMins,
         const Eigen::Vector4d &_segmentMaxs,
         const Eigen::Vector4d &_segmentRates         
        )
        : name_(_name)
{
  Eigen::Vector3d vector(_origin[0], _origin[1], _origin[2]);
  Eigen::Quaternion<double> q;
  q = Eigen::AngleAxis<double>(_segmentOffsets[0], Eigen::Vector3d(0.0, 0.0, 1.0));
  origin_ = Eigen::Affine3d::Identity();
  origin_.translate(vector);
  origin_ .rotate(q);

  segments_.emplace_back(_name + "_coxa",   _segmentLengths[0],              0.0, _segmentMins[0], _segmentMaxs[0], _segmentRates[0]);
  segments_.emplace_back(_name + "_femur",  _segmentLengths[1], _segmentOffsets[1], _segmentMins[1], _segmentMaxs[1], _segmentRates[1]);
  segments_.emplace_back(_name + "_tibia",  _segmentLengths[2], _segmentOffsets[2], _segmentMins[2], _segmentMaxs[2], _segmentRates[2]);
  segments_.emplace_back(_name + "_tarsus", _segmentLengths[3], _segmentOffsets[3], _segmentMins[3], _segmentMaxs[3], _segmentRates[3]);
}

void Leg::getAnglesFromPoint(const Eigen::Vector3d &_point, std::vector<double> &_angles) const
{
  _angles.clear();

  // lengths of each link
  double coxa_length  = segments_[0].getLength();
  double femur_length = segments_[1].getLength();
  double tibia_length = segments_[2].getLength();
  double tarsus_length= segments_[3].getLength();

  // offsets of each segment
  double coxa_offset = segments_[0].getOffset();
  double femur_offset = segments_[1].getOffset();
  double tibia_offset = segments_[2].getOffset();
  double tarsus_offset = segments_[3].getOffset();

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

  // Calculate position of tibia-tarsus segment 
  // assuming coxa is horizontal and tarsus is at selected slope
  double xr = std::abs(point[0]) - coxa_length - tarsus_length * cos(tarsus_slope);
  double yr = point[1];
  double zr = point[2] - tarsus_length * sin(tarsus_slope);

  // Form a triangle from the coxa-femur, femur-tibia, and tibia-tarsus segments
  // we can use the law of cosines to find its internal angles
 
  // length and slope of coxa-femur to tibia-tarsus edge
  double hyp = sqrt((xr*xr) + (zr*zr));
  double angle = atan(zr/xr);

  // femur segment
  double femur_numerator = (femur_length*femur_length) + (hyp*hyp) - (tibia_length*tibia_length);
  double femur_denominator = 2 * femur_length * hyp;
  double femur_angle = acos(femur_numerator / femur_denominator);
  femur_angle += angle;
  _angles.push_back(femur_angle - femur_offset);
  
  // tibia segment
  double tibia_numerator = (hyp*hyp) - (femur_length*femur_length) - (tibia_length*tibia_length);
  double tibia_denominator = 2 * femur_length * tibia_length;
  double tibia_angle = - acos(tibia_numerator / tibia_denominator);
  _angles.push_back(tibia_angle - tibia_offset);

  // tarsus segment
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
  double coxa_length  = segments_[0].getLength();
  double femur_length = segments_[1].getLength();
  double tibia_length = segments_[2].getLength();
  double tarsus_length= segments_[3].getLength();

  // offsets of each segment
  double coxa_offset = segments_[0].getOffset();
  double femur_offset = segments_[1].getOffset();
  double tibia_offset = segments_[2].getOffset();
  double tarsus_offset = segments_[3].getOffset();

  // Calculate location of foot in the plane of the leg (2D, yz plane(y horizontal, z up))
  // get angles of each link relative to y-axis (about x-axis)
  double coxa_slope = 0.0;

  double femur_slope  = coxa_slope;
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
  double coxa_angle = 0.0;
  coxa_angle += _angles[0] + coxa_offset;   
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
  angles[0] = segments_[0].getCurrentAngle();
  angles[1] = segments_[1].getCurrentAngle();
  angles[2] = segments_[2].getCurrentAngle();
  angles[3] = segments_[3].getCurrentAngle();

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

std::vector<Segment> Leg::getSegments() const { return segments_; }

void Leg::getSegmentLengths(Eigen::Vector4d &_lengths) const
{
  // Add results to vector
  _lengths[0] = segments_[0].getLength();
  _lengths[1] = segments_[1].getLength();
  _lengths[2] = segments_[2].getLength();
  _lengths[3] = segments_[3].getLength();
}

void Leg::getSegmentOffsets(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getOffset();
  _angles[1] = segments_[1].getOffset();
  _angles[2] = segments_[2].getOffset();
  _angles[3] = segments_[3].getOffset();
}

void Leg::getSegmentMinAngles(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getMinAngle();
  _angles[1] = segments_[1].getMinAngle();
  _angles[2] = segments_[2].getMinAngle();
  _angles[3] = segments_[3].getMinAngle();
}

void Leg::getSegmentMaxAngles(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getMaxAngle();
  _angles[1] = segments_[1].getMaxAngle();
  _angles[2] = segments_[2].getMaxAngle();
  _angles[3] = segments_[3].getMaxAngle();
}

void Leg::getSegmentCurrentAngles(Eigen::Vector4d &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getCurrentAngle();
  _angles[1] = segments_[1].getCurrentAngle();
  _angles[2] = segments_[2].getCurrentAngle();
  _angles[3] = segments_[3].getCurrentAngle();
}

std::map<std::string, double> Leg::getSegmentLengths() const
{
  // Get angles
  Eigen::Vector4d angles;
  getSegmentLengths(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getSegmentOffsets() const
{
  // Get angles
  Eigen::Vector4d angles;
  getSegmentOffsets(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getSegmentMinAngles() const
{
  // Get angles
  Eigen::Vector4d angles;
  getSegmentMinAngles(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getSegmentMaxAngles() const
{
  // Get angles
  Eigen::Vector4d angles;
  getSegmentMaxAngles(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, double> Leg::getSegmentCurrentAngles() const
{
  // Get angles
  Eigen::Vector4d angles;
  getSegmentCurrentAngles(angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

bool Leg::setSegmentLengths(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setLength(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setLength(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setLength(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setLength(tarsus_index->second);

  return result;
}

bool Leg::setSegmentOffsets(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setOffset(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setOffset(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setOffset(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setOffset(tarsus_index->second);

  return result;
}

bool Leg::setSegmentMinAngles(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setMinAngle(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setMinAngle(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setMinAngle(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setMinAngle(tarsus_index->second);

  return result;
}

bool Leg::setSegmentMaxAngles(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setMaxAngle(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setMaxAngle(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setMaxAngle(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setMaxAngle(tarsus_index->second);

  return result;
}
    
bool Leg::setSegmentCurrentAngles(const std::map<std::string, double> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setCurrentAngle(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setCurrentAngle(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setCurrentAngle(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setCurrentAngle(tarsus_index->second);

  return result;
}
