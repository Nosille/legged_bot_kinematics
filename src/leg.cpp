#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>

#include "leg.h"

/// @brief Constructor for Leg using eigen vectors
/// @param _name name of leg
/// @param _origin location of leg origin relative to body center in meters and radians (x, y, z, yaw)
/// @param _lengths length of each leg member from origin to end (coxa, femur, tibia, tarsus)
Leg::Leg(const std::string &_name,
         const Eigen::Vector4d &_origin,
         const Eigen::Vector4d &_lengths 
        )
        : name_(_name)
{
  origin_  = _origin;
  lengths_ = _lengths;
}

/// @brief Inverse Kinematics
/// @param _point x, y, z coordinates of foot in body coordinate system 
/// @param _angles return array of doubles containing joint angles in radians (coxa, femur, tibia, tarsus) 
void Leg::getAnglesFromPoint(const Eigen::Vector3d &_point, double (&_angles)[4])
{
  // lengths of each link
  double coxa_length = lengths_[0];
  double femur_length = lengths_[1];
  double tibia_length = lengths_[2];
  double tarsus_length = lengths_[3];

  // Transform from body coordinate system to leg coordinate system
  Eigen::Vector3d point;
  point[0] = _point[0] - origin_[0];
  point[1] = _point[1] - origin_[1];
  point[2] = _point[2] - origin_[2];
  double yaw = -atan(point[0]/point[1]);
  ROS_INFO_STREAM("yaw = " << yaw);

  Eigen::Quaternion<double> q;
  q = Eigen::AngleAxis<double>(yaw, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Vector3d vector = q.inverse() * point;

  // Coxa angle can be calculated at this point
  double coxa_angle = yaw - origin_[3];

  ROS_INFO_STREAM("xp = " << vector[0]);
  ROS_INFO_STREAM("yp = " << vector[1]);
  ROS_INFO_STREAM("zp = " << vector[2]);

  // Calculate position of tibia-tarsus joint 
  // assuming coxa is horizontal and tarsus is vertical
  double yr = abs(vector[1]) - coxa_length;
  double zr = vector[2] + tarsus_length;

  ROS_INFO_STREAM("yr = " << yr);
  ROS_INFO_STREAM("zr = " << zr);

  // Calculate joint angles for remaining two joints (tibia and femur) 
  // that yield the desired tibia-tarsus joint position
  double tibia_angle = (yr*yr) + (zr*zr) - (femur_length*femur_length) - (tibia_length*tibia_length);
  tibia_angle /= 2 * femur_length * tibia_length;
  tibia_angle = -1 * acos(tibia_angle);

  double femur_numerator = tibia_length * sin(tibia_angle);
  double femur_denominator = femur_length + tibia_length * cos(tibia_angle);
  double femur_angle = -1 * atan(femur_numerator / femur_denominator);
  femur_angle += atan(zr/yr);

  // Calculate tarsus joint angle required to make tarsus link vertical
  // now that we have the other angles
  double tarsus_angle = -1.5707963267948968 + femur_angle + tibia_angle;

  // Add results to map
  _angles[0] = coxa_angle;
  _angles[1] = femur_angle;
  _angles[2] = tibia_angle;
  _angles[3] = tarsus_angle;
}

/// @brief Inverse Kinematics
/// @param _point x, y, z coordinates of foot in body coordinate system 
/// @return map containing angles for each named joint (coxa, femur, tibia, tarsus)
std::map<std::string, double> Leg::getAnglesFromPoint(const Eigen::Vector3d &_point)
{
  // Get angles
  double angles[4];
  getAnglesFromPoint(_point, angles);    

  // Add results to map
  std::map<std::string, double> map;
  map["coxa"] = angles[0];
  map["femur"] = angles[1];
  map["tibia"] = angles[2];
  map["tarsus"] = angles[3];
  
  return map;
}

/// @brief Inverse Kinematics
/// @param _point x, y, z coordinates of foot in body coordinate system
/// @return python dictionary containing angles for each named joint (coxa, femur, tibia, tarsus)
pybind11::dict Leg::getAnglesFromPointPy(const Eigen::Vector3d &_point)
{
  std::map<std::string, double> map;
  map = getAnglesFromPoint(_point);
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

/// @brief Forward Kinematics
/// @param _angles map containing angles for each named joint (coxa, femur, tibia, tarsus)
/// @param _point return vector containing x, y, z coordinates of foot in body coordinate system
void Leg::getPointFromAngles(std::map<std::string,double> &_angles, Eigen::Vector3d &_point)
{
  // lengths of each link
  double coxa_length = lengths_[0];
  double femur_length = lengths_[1];
  double tibia_length = lengths_[2];
  double tarsus_length = lengths_[3];

  // Calculate location of foot in the plane of the leg (2D, yz plane(y horizontal, z up))
  // get angles of each link relative to y-axis (about x-axis)
  double coxa_angle = origin_[3];
  auto coxa_index = _angles.find("coxa");
  if (coxa_index != _angles.end()) 
    coxa_angle += coxa_index->second; 
    ROS_INFO_STREAM("yaw = " << coxa_angle);   

  double femur_angle  = 0.0;
  auto femur_index = _angles.find("femur");
  if (femur_index != _angles.end()) 
    femur_angle += femur_index->second; 

  double tibia_angle  = femur_angle;
  auto tibia_index = _angles.find("tibia");
  if (tibia_index != _angles.end()) 
    tibia_angle += tibia_index->second;  

  double tarsus_angle = tibia_angle; 
  auto tarsus_index = _angles.find("tarsus");
  if (tarsus_index != _angles.end()) 
    tarsus_angle += tarsus_index->second;

  // Position of foot relative to connection to robot at coxa
  double yp = coxa_length + 
              femur_length*cos(femur_angle) + 
              tibia_length*cos(tibia_angle) +
              tarsus_length*cos(tarsus_angle);

  double zp = 0.0 + 
              femur_length*sin(femur_angle) + 
              tibia_length*sin(tibia_angle) +
              tarsus_length*sin(tarsus_angle);

  ROS_INFO_STREAM("yp = " << yp);    
  ROS_INFO_STREAM("zp = " << zp);    
  
  // Create tranform from leg origin to end of leg
  if (origin_[1] < 0) yp *= -1;
  Eigen::Vector3d vector(0.0, yp, zp);
  Eigen::Quaternion<double> q;
  q = Eigen::AngleAxis<double>(coxa_angle, Eigen::Vector3d(0.0, 0.0, 1.0));

  // Shift point to body frame
  Eigen::Vector3d point = q * vector;
  _point[0] = origin_[0] + point[0];
  _point[1] = origin_[1] + point[1];
  _point[2] = origin_[2] + point[2];
}

/// @brief Forward Kinematics
/// @param dict python dictionary containing angles for each named joint (coxa, femur, tibia, tarsus)
/// @return vector containing x, y, z coordinates of foot in body coordinate system
Eigen::Vector3d Leg::getPointFromAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string,double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  // Get point
  Eigen::Vector3d point;
  getPointFromAngles(map, point);
  return point;
}

/// @brief Get name of Leg
/// @return name of leg
std::string Leg::getName() const { return name_; }

/// @brief Get origin of leg in body coordinate system
/// @return location of leg origin relative to body center in meters and radians (x, y, z, yaw)
Eigen::Vector4d Leg::getOrigin() const { return origin_; }

/// @brief Get joint distances used in kinematics
/// @return length between joints from origin to end (coxa, femur, tibia, tarsus)
Eigen::Vector4d Leg::getLengths() const { return lengths_; }
