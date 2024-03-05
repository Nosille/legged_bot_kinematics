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
  Eigen::Vector3d point;
  getPointFromAngles(map, point);
  return point;
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
        .def_property_readonly("name", &PyLeg::getName, "The leg's name")
        .def_property_readonly("origin", &PyLeg::getOrigin, "The leg's origin offset (x, y, z, yaw)")
        .def_property_readonly("lengths", &PyLeg::getLengths, "The leg's lengths (coxa, femur, tibia, tarsus)")
        .def_property_readonly("joints", &PyLeg::getJoints, "The leg's joints (coxa, femur, tibia, tarsus)")
        .def("__repr__", [](const PyLeg &leg) {
            return "<Leg named '" + leg.getName() + "'>";
        });

    pybind11::class_<PyJoint>(m, "Joint")
        .def(pybind11::init<std::string &,                    double &,                          double &,                         double &,                double &>(), 
                    pybind11::arg("name"), pybind11::arg("offset")=0.0, pybind11::arg("minAngle")=-1.5708, pybind11::arg("maxAngle")=1.5708, pybind11::arg("maxRate")=6)
        .def("setOffset", &PyJoint::setOffset, "Set offset of this joint from straight when angle is zero (rad)")
        .def("setMinAngle", &PyJoint::setMinAngle, "Set minimum allowed angle of joint (rad)")
        .def("setMaxAngle", &PyJoint::setMaxAngle, "Set maximum allowed angle of joint (rad)")
        .def("setMaxRate", &PyJoint::setMaxRate, "Set maximum allowed angular rate of joint (rad/sec)")
        .def("setCurrentAngle", &PyJoint::setCurrentAngle, "Set current angle of joint (rad)")
        .def_property_readonly("name", &PyJoint::getName, "The joints's name")
        .def_property_readonly("getOffset", &PyJoint::getOffset, "Get offset of joint from straight at zero angle (rad)")
        .def_property_readonly("getMinAngle", &PyJoint::getMinAngle, "Get minimum allowed angle of this joint (rad)")
        .def_property_readonly("getMaxAngle", &PyJoint::getMaxAngle, "Get maximum allowed angle of this joint (rad)")
        .def_property_readonly("getMaxRate", &PyJoint::getMaxRate, "Get maximum allowed angular rate of this joint (rad/sec)")
        .def_property_readonly("getCurrentAngle", &PyJoint::getCurrentAngle, "Get current angle of this joint (rad)")
        .def("__repr__", [](const PyJoint &joint) {
            return "<Joint named '" + joint.getName() + "'>";
        });
};
