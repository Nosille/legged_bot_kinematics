#include "python_bridge.h"

pybind11::dict PyLeg::getAnglesFromPointPy(const Eigen::Vector3d &_point)
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

Eigen::Vector3d PyLeg::getPointFromAnglesPy(const pybind11::dict &_dict)
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
  return getPointFromAngles(map);
}

bool PyLeg::setJointLengthsPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setJointLengths(map);
}

bool PyLeg::setJointOffsetsPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setJointOffsets(map);
}

bool PyLeg::setJointMinAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setJointMinAngles(map);
}

bool PyLeg::setJointMaxAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setJointMaxAngles(map);
}

bool PyLeg::setJointCurrentAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setJointCurrentAngles(map);
}

pybind11::dict PyLeg::getJointLengthsPy()
{
  std::map<std::string, double> map;
  map = getJointLengths();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getJointOffsetsPy()
{
  std::map<std::string, double> map;
  map = getJointOffsets();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getJointMinAnglesPy()
{
  std::map<std::string, double> map;
  map = getJointMinAngles();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getJointMaxAnglesPy()
{
  std::map<std::string, double> map;
  map = getJointMaxAngles();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getJointCurrentAnglesPy()
{
  std::map<std::string, double> map;
  map = getJointCurrentAngles();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

PYBIND11_MODULE(leg_kinematics, m) 
{
    pybind11::class_<PyBot>(m, "Bot")
        .def(pybind11::init<const std::string &, const std::list<std::string> &, const std::list<Eigen::Vector3d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &>(), 
                          pybind11::arg("name"),        pybind11::arg("legIds"),        pybind11::arg("legOrigins"),        pybind11::arg("legLengths"),      pybind11::arg("jointOffsets"),         pybind11::arg("jointMins"),         pybind11::arg("jointMaxs"),           pybind11::arg("jointRates"))
        .def("setLegPosition", static_cast<std::vector<double> (PyBot::*)(int, const Eigen::Vector3d &)>(&PyBot::setLegPosition), pybind11::arg("index"), pybind11::arg("point"), "Send Point get Angles")
        .def("setLegPosition", static_cast<std::vector<double> (PyBot::*)(const std::string &, const Eigen::Vector3d &)>(&PyBot::setLegPosition), pybind11::arg("id"), pybind11::arg("point"), "Send Point get Angles")
        .def_property_readonly("name", &PyBot::getName, "The bot's name")
        .def("__repr__", [](const PyBot &bot) {
            return "<Bot named '" + bot.getName() + "'>";
        });

    pybind11::class_<PyLeg>(m, "Leg")
        .def(pybind11::init<const std::string &, const Eigen::Vector3d &,  const Eigen::Vector4d &,       const Eigen::Vector4d &,    const Eigen::Vector4d &,    const Eigen::Vector4d &,  const Eigen::Vector4d &>(), 
                          pybind11::arg("name"), pybind11::arg("origin"), pybind11::arg("lengths"), pybind11::arg("jointOffsets"), pybind11::arg("jointMins"), pybind11::arg("jointMaxs"), pybind11::arg("jointRates"))
        .def("getAnglesFromPoint", &PyLeg::getAnglesFromPointPy, "Send Point get Angles")
        .def("getPointFromAngles", &PyLeg::getPointFromAnglesPy, "Send Angles get Point")
        .def("setJointLengths", &PyLeg::setJointLengthsPy, "Set distance to next joint (meters)")
        .def("setJointOffsets", &PyLeg::setJointOffsetsPy, "Set offset of joints from straight when angle equals zero (rad)")
        .def("setJointMinAngles", &PyLeg::setJointMinAnglesPy, "Set min angles of joints (rad)")
        .def("setJointMaxAngles", &PyLeg::setJointMaxAnglesPy, "Set max angles of joints (rad)")
        .def("setJointCurrentAngles", &PyLeg::setJointCurrentAnglesPy, "Set current angles of joints (rad)")
        .def_property_readonly("name", &PyLeg::getName, "The leg's name")
        .def_property_readonly("origin", &PyLeg::getOrigin, "The leg's origin offset (x, y, z, yaw)")
        .def_property_readonly("joints", &PyLeg::getJoints, "The leg's joints (coxa, femur, tibia, tarsus)")
        .def_property_readonly("jointLengths", &PyLeg::getJointLengthsPy, "The distance to the next joint (coxa, femur, tibia, tarsus)")
        .def_property_readonly("jointOffsets", &PyLeg::getJointOffsetsPy, "The leg joint's offset from straight at zero angle (coxa, femur, tibia, tarsus)")
        .def_property_readonly("jointMinAngles", &PyLeg::getJointMinAnglesPy, "Get min angles of all the joints (rad)")
        .def_property_readonly("jointMaxAngles", &PyLeg::getJointMaxAnglesPy, "Get max angles of all the joints (rad)")
        .def_property_readonly("jointCurrentAngles", &PyLeg::getJointCurrentAnglesPy, "Get current angle of all the joints (rad)")
        .def("__repr__", [](const PyLeg &leg) {
            return "<Leg named '" + leg.getName() + "'>";
        });

    pybind11::class_<PyJoint>(m, "Joint")
        .def(pybind11::init<std::string &,                double &,                    double &,                          double &,                         double &,                double &>(), 
                    pybind11::arg("name"), pybind11::arg("length"), pybind11::arg("offset")=0.0, pybind11::arg("minAngle")=-1.5708, pybind11::arg("maxAngle")=1.5708, pybind11::arg("maxRate")=6)
        .def("setLength", &PyJoint::setLength, "Set distance from this joint to the next joint (meter)")
        .def("setOffset", &PyJoint::setOffset, "Set offset of this joint from straight when angle is zero (rad)")
        .def("setMinAngle", &PyJoint::setMinAngle, "Set minimum allowed angle of joint (rad)")
        .def("setMaxAngle", &PyJoint::setMaxAngle, "Set maximum allowed angle of joint (rad)")
        .def("setMaxRate", &PyJoint::setMaxRate, "Set maximum allowed angular rate of joint (rad/sec)")
        .def("setCurrentAngle", &PyJoint::setCurrentAngle, "Set current angle of joint (rad)")
        .def_property_readonly("name", &PyJoint::getName, "The joints's name")
        .def_property_readonly("length", &PyJoint::getLength, "Get distance from this joint to the next joint (meter)")
        .def_property_readonly("offset", &PyJoint::getOffset, "Get offset of joint from straight at zero angle (rad)")
        .def_property_readonly("minAngle", &PyJoint::getMinAngle, "Get minimum allowed angle of this joint (rad)")
        .def_property_readonly("maxAngle", &PyJoint::getMaxAngle, "Get maximum allowed angle of this joint (rad)")
        .def_property_readonly("maxRate", &PyJoint::getMaxRate, "Get maximum allowed angular rate of this joint (rad/sec)")
        .def_property_readonly("currentAngle", &PyJoint::getCurrentAngle, "Get current angle of this joint (rad)")
        .def("__repr__", [](const PyJoint &joint) {
            return "<Joint named '" + joint.getName() + "'>";
        });
};
