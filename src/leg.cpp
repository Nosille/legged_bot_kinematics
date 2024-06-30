
#include <iterator>
#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "leg.h"

Leg::Leg(const std::string &_name, 
         const Eigen::Affine3f &_origin,
         const std::vector<Segment> &_segments,
         uint16_t _anglePrecisionFactor,
         uint16_t _lengthPrecisionFactor
        )
        : name_(_name)
        , anglePrecisionFactor_(_anglePrecisionFactor)
        , lengthPrecisionFactor_(_anglePrecisionFactor)
{
  origin_  = _origin;
  originInv_  = _origin.inverse();

  for(Segment segment : _segments)
  {
    segment.setAnglePrecisionFactor(_anglePrecisionFactor);
    segment.setAnglePrecisionFactor(_lengthPrecisionFactor);
    segments_.push_back(segment);
  }
  
  while(segments_.size() < 4)
  {
    segments_.emplace_back(_name + std::to_string(segments_.size()), 0.0, 0.0, -M_PI, M_PI, 10.0, _anglePrecisionFactor, _lengthPrecisionFactor);
  }
}

Leg::Leg(const std::string &_name,
         const Eigen::Affine3f &_origin,
         const Eigen::Vector4f &_segmentLengths,
         const Eigen::Vector4f &_segmentOffsets,
         const Eigen::Vector4f &_segmentMins,
         const Eigen::Vector4f &_segmentMaxs,
         const Eigen::Vector4f &_segmentRates,
         uint16_t _anglePrecisionFactor,
         uint16_t _lengthPrecisionFactor
        )
        : name_(_name)
        , anglePrecisionFactor_(_anglePrecisionFactor)
        , lengthPrecisionFactor_(_anglePrecisionFactor)
{
  origin_  = _origin;
  originInv_  = _origin.inverse();

  segments_.emplace_back(_name + "_coxa",   _segmentLengths[0], _segmentOffsets[0], _segmentMins[0], _segmentMaxs[0], _segmentRates[0], _anglePrecisionFactor, _lengthPrecisionFactor);
  segments_.emplace_back(_name + "_femur",  _segmentLengths[1], _segmentOffsets[1], _segmentMins[1], _segmentMaxs[1], _segmentRates[1], _anglePrecisionFactor, _lengthPrecisionFactor);
  segments_.emplace_back(_name + "_tibia",  _segmentLengths[2], _segmentOffsets[2], _segmentMins[2], _segmentMaxs[2], _segmentRates[2], _anglePrecisionFactor, _lengthPrecisionFactor);
  segments_.emplace_back(_name + "_tarsus", _segmentLengths[3], _segmentOffsets[3], _segmentMins[3], _segmentMaxs[3], _segmentRates[3], _anglePrecisionFactor, _lengthPrecisionFactor);
}

Leg::Leg(const std::string &_name,
         const Eigen::VectorXf &_origin,
         const Eigen::Vector4f &_segmentLengths,
         const Eigen::Vector4f &_segmentOffsets,
         const Eigen::Vector4f &_segmentMins,
         const Eigen::Vector4f &_segmentMaxs,
         const Eigen::Vector4f &_segmentRates,
         uint16_t _anglePrecisionFactor,
         uint16_t _lengthPrecisionFactor         
        )
        : name_(_name)
        , anglePrecisionFactor_(_anglePrecisionFactor)
        , lengthPrecisionFactor_(_anglePrecisionFactor)
{
  Eigen::Vector3f vector(_origin[0], _origin[1], _origin[2]);
  Eigen::Quaternion<float> q;
  q = Eigen::AngleAxis<float>(_origin[5], Eigen::Vector3f(0.0, 0.0, 1.0)) *   // Need to confirm these are in correct order
      Eigen::AngleAxis<float>(_origin[4], Eigen::Vector3f(0.0, 1.0, 0.0)) *
      Eigen::AngleAxis<float>(_origin[3], Eigen::Vector3f(1.0, 0.0, 0.0));
  Eigen::Affine3f origin = Eigen::Affine3f::Identity();
  origin.translate(vector);
  origin.rotate(q);
  origin_  = origin;
  originInv_ = origin.inverse();

  segments_.emplace_back(_name + "_coxa",   _segmentLengths[0],                0.0, _segmentMins[0], _segmentMaxs[0], _segmentRates[0], _anglePrecisionFactor, _lengthPrecisionFactor);
  segments_.emplace_back(_name + "_femur",  _segmentLengths[1], _segmentOffsets[1], _segmentMins[1], _segmentMaxs[1], _segmentRates[1], _anglePrecisionFactor, _lengthPrecisionFactor);
  segments_.emplace_back(_name + "_tibia",  _segmentLengths[2], _segmentOffsets[2], _segmentMins[2], _segmentMaxs[2], _segmentRates[2], _anglePrecisionFactor, _lengthPrecisionFactor);
  segments_.emplace_back(_name + "_tarsus", _segmentLengths[3], _segmentOffsets[3], _segmentMins[3], _segmentMaxs[3], _segmentRates[3], _anglePrecisionFactor, _lengthPrecisionFactor);
}

Leg::Leg(const std::string &_name,
         const float _origin[6],
         uint8_t _count,
         const float *_segmentLengths,
         const float *_segmentOffsets,
         const float *_segmentMins,
         const float *_segmentMaxs,
         const float *_segmentRates,
         uint16_t _anglePrecisionFactor,
         uint16_t _lengthPrecisionFactor        
        )
        : name_(_name)
        , anglePrecisionFactor_(_anglePrecisionFactor)
        , lengthPrecisionFactor_(_anglePrecisionFactor)
{
  Eigen::Vector3f vector(_origin[0], _origin[1], _origin[2]);
  Eigen::Quaternion<float> q;
  q = Eigen::AngleAxis<float>(_origin[5], Eigen::Vector3f(0.0, 0.0, 1.0)) *   // Need to confirm these are in correct order
      Eigen::AngleAxis<float>(_origin[4], Eigen::Vector3f(0.0, 1.0, 0.0)) *
      Eigen::AngleAxis<float>(_origin[3], Eigen::Vector3f(1.0, 0.0, 0.0));
  Eigen::Affine3f origin = Eigen::Affine3f::Identity();
  origin.translate(vector);
  origin .rotate(q);
  origin_  = origin;
  originInv_ = origin.inverse();

  if(_count <= 4)
  {
    if(_count > 0) segments_.emplace_back(_name + "_coxa",   _segmentLengths[0], _segmentOffsets[0], _segmentMins[0], _segmentMaxs[0], _segmentRates[0], _anglePrecisionFactor, _lengthPrecisionFactor);
    if(_count > 1) segments_.emplace_back(_name + "_femur",  _segmentLengths[1], _segmentOffsets[1], _segmentMins[1], _segmentMaxs[1], _segmentRates[1], _anglePrecisionFactor, _lengthPrecisionFactor);
    if(_count > 2) segments_.emplace_back(_name + "_tibia",  _segmentLengths[2], _segmentOffsets[2], _segmentMins[2], _segmentMaxs[2], _segmentRates[2], _anglePrecisionFactor, _lengthPrecisionFactor);
    if(_count > 3) segments_.emplace_back(_name + "_tarsus", _segmentLengths[3], _segmentOffsets[3], _segmentMins[3], _segmentMaxs[3], _segmentRates[3], _anglePrecisionFactor, _lengthPrecisionFactor);
  }
  else
  {
    segments_.emplace_back(_name + std::to_string(0), _segmentLengths[0], 0.0, _segmentMins[0], _segmentMaxs[0], _segmentRates[0]);
    for(int i = 1; i < _count; i++)
    {
      segments_.emplace_back(_name + std::to_string(i), _segmentLengths[i], _segmentOffsets[i], _segmentMins[i], _segmentMaxs[i], _segmentRates[i]);
    }
  }
}

void Leg::calcAnglesFromPoint(const Eigen::Vector<int32_t, 3> &_point, std::vector<int32_t> &_angles) const
{
  _angles.clear();

  uint16_t anglePrecisionFactor = segments_[0].getAnglePrecisionFactor();
  uint16_t lengthPrecisionFactor = segments_[0].getLengthPrecisionFactor();

  // lengths of each link
  int32_t coxa_length  = segments_[0].getLengthAsInt();
  int32_t femur_length = segments_[1].getLengthAsInt();
  int32_t tibia_length = segments_[2].getLengthAsInt();
  int32_t tarsus_length= segments_[3].getLengthAsInt();

  // offsets of each segment
  int32_t coxa_offset = segments_[0].getAngleOffsetAsInt();
  int32_t femur_offset = segments_[1].getAngleOffsetAsInt();
  int32_t tibia_offset = segments_[2].getAngleOffsetAsInt();
  int32_t tarsus_offset = segments_[3].getAngleOffsetAsInt();

  // Transform point from body coordinate system to leg coordinate system
  Eigen::Matrix<int32_t, 4, 4> originInv {
              {(int32_t)originInv_(0,0)*lengthPrecisionFactor, (int32_t)originInv_(0,1)*lengthPrecisionFactor, (int32_t)originInv_(0,2)*lengthPrecisionFactor, (int32_t)(originInv_(0,3)*lengthPrecisionFactor)},
              {(int32_t)originInv_(1,0)*lengthPrecisionFactor, (int32_t)originInv_(1,1)*lengthPrecisionFactor, (int32_t)originInv_(1,2)*lengthPrecisionFactor, (int32_t)(originInv_(1,3)*lengthPrecisionFactor)},
              {(int32_t)originInv_(2,0)*lengthPrecisionFactor, (int32_t)originInv_(2,1)*lengthPrecisionFactor, (int32_t)originInv_(2,2)*lengthPrecisionFactor, (int32_t)(originInv_(2,3)*lengthPrecisionFactor)},
              {(int32_t)originInv_(3,0)*lengthPrecisionFactor, (int32_t)originInv_(3,1)*lengthPrecisionFactor, (int32_t)originInv_(3,2)*lengthPrecisionFactor, (int32_t)(originInv_(3,3)*lengthPrecisionFactor)}};

  Eigen::Vector<int32_t, 4> pointBody(_point[0], _point[1], _point[2], lengthPrecisionFactor);
  Eigen::Vector<int32_t, 4> pointLeg = originInv * pointBody;
  pointLeg /= lengthPrecisionFactor;

  // Coxa angle can be calculated at this point
  float coxa_angle = atan(float(pointLeg[1]) / float(pointLeg[0]));
  _angles.push_back(coxa_angle * anglePrecisionFactor - coxa_offset);
  // while(coxa_out > +PI/2) coxa_out -= PI;
  // while(coxa_out < -PI/2) coxa_out += PI;

  int32_t hypInXY = pointLeg[1]*pointLeg[1] + pointLeg[0]*pointLeg[0];
  hypInXY = isqrt32(hypInXY);
  int32_t cosInXY = pointLeg[0]*lengthPrecisionFactor/hypInXY;
  int32_t sinInXY = pointLeg[1]*lengthPrecisionFactor/hypInXY;
  Eigen::Matrix<int32_t, 4, 4> rotationZ{
                                  { cosInXY,  sinInXY,                     0,                    0},
                                  {-sinInXY,  cosInXY,                     0,                    0},
                                  {       0,        0, lengthPrecisionFactor,                    0},
                                  {       0,        0,                     0, lengthPrecisionFactor}};

  pointLeg = rotationZ * pointLeg / lengthPrecisionFactor;
  // Serial.print(pointLeg[0]); Serial.print(":");Serial.print(pointLeg[1]); Serial.print(":");Serial.print(pointLeg[2]);Serial.print(":");Serial.println(pointLeg[3]);

  // Select tarsus slope
  float tarsus_slope = -M_PI/2;

  // Calculate position of tibia-tarsus joint
  // assuming coxa is horizontal and tarsus is at selected slope
  int32_t xr = std::abs(pointLeg[0]) - coxa_length - tarsus_length * cos(tarsus_slope);
  int32_t yr = pointLeg[1];
  int32_t zr = pointLeg[2] - tarsus_length * sin(tarsus_slope);

  // Form a triangle from the coxa-femur, femur-tibia, and tibia-tarsus joints
  // we can use the law of cosines to find its internal angles
 
  // length and slope of coxa-femur to tibia-tarsus edge
  int32_t hypInXZ = xr*xr + zr*zr;
  hypInXZ = isqrt32(hypInXZ);
  float angle = atan((float)zr/((float)xr));

  // femur segment
  float femur_numerator = (femur_length*femur_length) + (hypInXZ*hypInXZ) - (tibia_length*tibia_length);
  float femur_denominator = (2 * femur_length * hypInXZ);
  float femur_angle = acos(femur_numerator / femur_denominator);
  femur_angle += angle;
  _angles.push_back((int32_t)(femur_angle * anglePrecisionFactor) - femur_offset);

  // tibia segment
  float tibia_numerator = (hypInXZ*hypInXZ) - (femur_length*femur_length) - (tibia_length*tibia_length);
  float tibia_denominator = 2 * femur_length * tibia_length;
  float tibia_angle = - acos(tibia_numerator / (tibia_denominator));
  _angles.push_back((int32_t)(tibia_angle * anglePrecisionFactor) - tibia_offset);

  // tarsus segment
  float tarsus_angle = tarsus_slope - femur_angle - tibia_angle;
  _angles.push_back((int32_t)(tarsus_angle * anglePrecisionFactor) - tarsus_offset);
}

void Leg::calcAnglesFromPoint(const Eigen::Vector3f &_point, std::vector<float> &_angles) const
{
  _angles.clear();

  // lengths of each link
  float coxa_length  = segments_[0].getLength();
  float femur_length = segments_[1].getLength();
  float tibia_length = segments_[2].getLength();
  float tarsus_length= segments_[3].getLength();

  // offsets of each segment
  float coxa_offset = segments_[0].getAngleOffset();
  float femur_offset = segments_[1].getAngleOffset();
  float tibia_offset = segments_[2].getAngleOffset();
  float tarsus_offset = segments_[3].getAngleOffset();

  // Transform point from body coordinate system to leg coordinate system
  Eigen::Vector3f pointLeg;
  pointLeg = originInv_ * _point;

  // Coxa angle can be calculated at this point
  float coxa_angle = atan(pointLeg[1]/pointLeg[0]);
  // while(coxa_out > +PI/2) coxa_out -= PI;
  // while(coxa_out < -PI/2) coxa_out += PI;
  
  _angles.push_back(coxa_angle - coxa_offset);
  Eigen::Quaternion<float> q;
  q = Eigen::AngleAxis<float>(-coxa_angle, Eigen::Vector3f(0.0, 0.0, 1.0));
  Eigen::Matrix<float, 3, 3> rotationZ(q);
  rotationZ = Eigen::AngleAxis<float>(-coxa_angle, Eigen::Vector3f(0.0, 0.0, 1.0));
  pointLeg = q * pointLeg;

  // Select tarsus slope
  float tarsus_slope = -M_PI/2;

  // Calculate position of tibia-tarsus segment 
  // assuming coxa is horizontal and tarsus is at selected slope
  float xr = std::abs(pointLeg[0]) - coxa_length - tarsus_length * cos(tarsus_slope);
  float yr = pointLeg[1];
  float zr = pointLeg[2] - tarsus_length * sin(tarsus_slope);

  // Form a triangle from the coxa-femur, femur-tibia, and tibia-tarsus segments
  // we can use the law of cosines to find its internal angles
 
  // length and slope of coxa-femur to tibia-tarsus edge
  float hyp = sqrt((xr*xr) + (zr*zr));
  float angle = atan(zr/xr);

  // femur segment
  float femur_numerator = (femur_length*femur_length) + (hyp*hyp) - (tibia_length*tibia_length);
  float femur_denominator = 2 * femur_length * hyp;
  float femur_angle = acos(femur_numerator / femur_denominator);
  femur_angle += angle;
  _angles.push_back(femur_angle - femur_offset);
  
  // tibia segment
  float tibia_numerator = (hyp*hyp) - (femur_length*femur_length) - (tibia_length*tibia_length);
  float tibia_denominator = 2 * femur_length * tibia_length;
  float tibia_angle = - acos(tibia_numerator / tibia_denominator);
  _angles.push_back(tibia_angle - tibia_offset);

  // tarsus segment
  float tarsus_angle = tarsus_slope - femur_angle - tibia_angle;
  _angles.push_back(tarsus_angle - tarsus_offset);
}

std::map<std::string, float> Leg::calcAnglesFromPoint(const Eigen::Vector3f &_point) const
{
  // Get angles
  std::vector<float> angles;
  calcAnglesFromPoint(_point, angles);    

  // Add results to map
  std::map<std::string, float> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];
  
  return map;
}

void Leg::calcPointFromAngles(const std::vector<float> &_angles, Eigen::Vector3f &_point) const
{
  // lengths and offsets of each segment
  float lengths[_angles.size()];
  float offsets[_angles.size()];  
  for(int i = 0; i < _angles.size(); i++)
  {
    lengths[i]  = segments_[i].getLength();
    offsets[i]  = segments_[i].getAngleOffset();
  }
  
  // Calculate location of foot in the plane of the leg (2D, yz plane(y horizontal, z up))
  // get angles of each link relative to y-axis (about x-axis)
  float slopes[_angles.size()];
  slopes[0] = 0.0f;
  for(int i = 1; i < _angles.size(); i++)
  {
    slopes[i] = slopes[i - 1];
    slopes[i] += _angles[i] + offsets[i]; 
  }

  // Position of foot relative to connection to robot at coxa
  float xp = 0.0f;
  float yp = 0.0f;
  float zp = 0.0f;
  for(int i = 0; i < _angles.size(); i++)
  {
    xp += lengths[i]*cos(slopes[i]);
    zp += lengths[i]*sin(slopes[i]);
  }

  // Create tranform from leg origin to end of leg
  Eigen::Vector3f vector(xp, yp, zp);

  Eigen::Quaternion<float> q;
  float coxa_angle = 0.0;
  coxa_angle += _angles[0] + offsets[0]; 
  q = Eigen::AngleAxis<float>(coxa_angle, Eigen::Vector3f(0.0, 0.0, 1.0));
  Eigen::Vector3f point = q * vector;

  // Shift point to body frame
  _point = origin_ * point;
}

Eigen::Vector3f Leg::calcPointFromAngles(const Eigen::Vector4f &_angles) const
{
  const std::vector<float> angles{_angles[0], _angles[1], _angles[2], _angles[3]};
  Eigen::Vector3f point;

  calcPointFromAngles(angles, point);
  
  return point;
}

Eigen::Vector3f Leg::calcPointFromAngles(const std::map<std::string, float> &_angles) const
{
  Eigen::Vector3f point;
  
  // Extract angles
  float coxa_angle = 0.0;
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    coxa_angle += coxa_index->second; 

  float femur_angle  = 0.0;
  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    femur_angle += femur_index->second; 

  float tibia_angle  = 0.0;
  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    tibia_angle += tibia_index->second;  

  float tarsus_angle = 0.0; 
  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    tarsus_angle += tarsus_index->second;

  // Add results to vector
  std::vector<float> angles(4);
  angles[0] = coxa_angle;
  angles[1] = femur_angle;
  angles[2] = tibia_angle;
  angles[3] = tarsus_angle;

  // Calculate point
  calcPointFromAngles(angles, point);  

  return point; 
}

const Eigen::Vector3f Leg::getPositionCurrent() const
{
  // Add results to map
  Eigen::Vector4f angles;
  angles[0] = segments_[0].getAngleCurrent();
  angles[1] = segments_[1].getAngleCurrent();
  angles[2] = segments_[2].getAngleCurrent();
  angles[3] = segments_[3].getAngleCurrent();

  return Leg::calcPointFromAngles(angles);
}

const Eigen::Vector3f Leg::getPositionHome() const
{
  return homePosition_;
}

bool Leg::setPositionCurrent(const Eigen::Vector3f &_position)
{
  std::vector<float> angles;
  calcAnglesFromPoint(_position, angles);

  for(uint8_t i = 0; i > segments_.size(); i++)
  {
    segments_[i].setAngleCurrent(angles[i]);
  }

  return true;
}

bool Leg::setPositionHome(const Eigen::Vector3f &_position)
{
  homePosition_ = _position;

  return true;
}

Eigen::Vector3f Leg::calcDistance(const Eigen::Vector3f &_point)
{
  Eigen::Vector3f distance;
  Eigen::Vector3f currentPoint = getPositionCurrent();

  distance[0] = currentPoint[0] - _point[0];
  distance[1] = currentPoint[1] - _point[1];
  distance[2] = currentPoint[2] - _point[2];

  return distance;
}

float Leg::calcDistancexyz(const Eigen::Vector3f &_point)
{
  Eigen::Vector3f vector = calcDistance(_point);

  float distance = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[1]*vector[1]);

  return distance;
}

float Leg::calcDistancexy(const Eigen::Vector3f &_point)
{
  Eigen::Vector3f vector = calcDistance(_point);

  float distance = sqrt(vector[0]*vector[0] + vector[1]*vector[1]);

  return distance;
}

float Leg::calcDistancexz(const Eigen::Vector3f &_point)
{
  Eigen::Vector3f vector = calcDistance(_point);

  float distance = sqrt(vector[0]*vector[0] + vector[2]*vector[2]);

  return distance;
}

float Leg::calcDistanceyz(const Eigen::Vector3f &_point)
{
  Eigen::Vector3f vector = calcDistance(_point);

  float distance = sqrt(vector[1]*vector[1] + vector[2]*vector[2]);

  return distance;
}

std::string Leg::getName() const { return name_; }

Eigen::Affine3f Leg::getOrigin() const { return origin_; }

std::vector<Segment> Leg::getSegments() const { return segments_; }

void Leg::getSegmentLengths(Eigen::Vector4f &_lengths) const
{
  // Add results to vector
  _lengths[0] = segments_[0].getLength();
  _lengths[1] = segments_[1].getLength();
  _lengths[2] = segments_[2].getLength();
  _lengths[3] = segments_[3].getLength();
}

void Leg::getSegmentAnglesOffset(Eigen::Vector4f &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getAngleOffset();
  _angles[1] = segments_[1].getAngleOffset();
  _angles[2] = segments_[2].getAngleOffset();
  _angles[3] = segments_[3].getAngleOffset();
}

void Leg::getSegmentAnglesCurrent(Eigen::Vector4f &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getAngleCurrent();
  _angles[1] = segments_[1].getAngleCurrent();
  _angles[2] = segments_[2].getAngleCurrent();
  _angles[3] = segments_[3].getAngleCurrent();
}

void Leg::getSegmentAnglesMin(Eigen::Vector4f &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getAngleMin();
  _angles[1] = segments_[1].getAngleMin();
  _angles[2] = segments_[2].getAngleMin();
  _angles[3] = segments_[3].getAngleMin();
}

void Leg::getSegmentAnglesMax(Eigen::Vector4f &_angles) const
{
  // Add results to vector
  _angles[0] = segments_[0].getAngleMax();
  _angles[1] = segments_[1].getAngleMax();
  _angles[2] = segments_[2].getAngleMax();
  _angles[3] = segments_[3].getAngleMax();
}

std::map<std::string, float> Leg::getSegmentLengths() const
{
  // Get angles
  Eigen::Vector4f angles;
  getSegmentLengths(angles);    

  // Add results to map
  std::map<std::string, float> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, float> Leg::getSegmentAnglesOffset() const
{
  // Get angles
  Eigen::Vector4f angles;
  getSegmentAnglesOffset(angles);    

  // Add results to map
  std::map<std::string, float> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, float> Leg::getSegmentAnglesCurrent() const
{
  // Get angles
  Eigen::Vector4f angles;
  getSegmentAnglesCurrent(angles);    

  // Add results to map
  std::map<std::string, float> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, float> Leg::getSegmentAnglesMin() const
{
  // Get angles
  Eigen::Vector4f angles;
  getSegmentAnglesMin(angles);    

  // Add results to map
  std::map<std::string, float> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

std::map<std::string, float> Leg::getSegmentAnglesMax() const
{
  // Get angles
  Eigen::Vector4f angles;
  getSegmentAnglesMax(angles);    

  // Add results to map
  std::map<std::string, float> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];

  return map;
}

uint16_t Leg::getAnglePrecisionFactor() const
{
  return anglePrecisionFactor_;
}

uint16_t Leg::getLengthPrecisionFactor() const
{
  return lengthPrecisionFactor_;
}

bool Leg::setSegmentLengths(const std::map<std::string, float> &_angles)
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

bool Leg::setSegmentAnglesOffset(const std::map<std::string, float> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setAngleOffset(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setAngleOffset(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setAngleOffset(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setAngleOffset(tarsus_index->second);

  return result;
}
    
bool Leg::setSegmentAnglesCurrent(const std::map<std::string, float> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setAngleCurrent(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setAngleCurrent(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setAngleCurrent(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setAngleCurrent(tarsus_index->second);

  return result;
}

bool Leg::setSegmentAnglesMin(const std::map<std::string, float> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setAngleMin(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setAngleMin(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setAngleMin(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setAngleMin(tarsus_index->second);

  return result;
}

bool Leg::setSegmentAnglesMax(const std::map<std::string, float> &_angles)
{
  bool result = true;
 
  // Extract angles
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    result *= segments_[0].setAngleMax(coxa_index->second); 

  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    result *= segments_[1].setAngleMax(femur_index->second); 

  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    result *= segments_[2].setAngleMax(tibia_index->second);  

  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    result *= segments_[3].setAngleMax(tarsus_index->second);

  return result;
}

bool Leg::setAnglePrecisionFactor(uint16_t _factor) 
{ 
  anglePrecisionFactor_ = _factor; 
  
  for(Segment segment : segments_)
  {
    segment.setAnglePrecisionFactor(anglePrecisionFactor_);
  }
  return true;
}

bool Leg::setLengthPrecisionFactor(uint16_t _factor) 
{ 
  lengthPrecisionFactor_ = _factor; 

  for(Segment segment : segments_)
  {
    segment.setLengthPrecisionFactor(lengthPrecisionFactor_);
  }  
  return true;
}

uint32_t Leg::isqrt32(uint32_t n)
{
        uint32_t root;
        uint32_t remainder;
        uint32_t  place;

        root = 0;
        remainder = n;
        place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

        while (place > remainder)
        place = place >> 2;
        while (place)
        {
                if (remainder >= root + place)
                {
                        remainder = remainder - root - place;
                        root = root + (place << 1);
                }
                root = root >> 1;
                place = place >> 2;
        }
        return root;
}