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

PYBIND11_MODULE(kinematics, m) 
{
    pybind11::class_<PyBot>(m, "Bot")
        .def(pybind11::init<const std::string &, const std::string &, const std::list<std::string> &, const double &>(), pybind11::arg("name"), pybind11::arg("frame_id"), pybind11::arg("legIds"), pybind11::arg("wait_for_tf_delay") = 10)
        .def_property_readonly("name", &PyBot::getName, "The bot's name")
        .def("__repr__", [](const PyBot &bot) {
            return "<Bot named '" + bot.getName() + "'>";
        });

    pybind11::class_<PyLeg>(m, "Leg")
        .def(pybind11::init<const std::string &, const Eigen::Vector4d &, const Eigen::Vector4d &>(), pybind11::arg("name"), pybind11::arg("origin"), pybind11::arg("lengths"))
        .def("getAnglesFromPoint", &PyLeg::getAnglesFromPointPy, "Send Point get Angles")
        .def("getPointFromAngles", &PyLeg::getPointFromAnglesPy, "Send Angles get Point")
        .def_property_readonly("name", &Leg::getName, "The leg's name")
        .def_property_readonly("origin", &Leg::getOrigin, "The leg's origin offset (x, y, z, yaw)")
        .def_property_readonly("lengths", &Leg::getLengths, "The leg's lengths (coxa, femur, tibia, tarsus)")
        .def("__repr__", [](const Leg &leg) {
            return "<Leg named '" + leg.getName() + "'>";
        });

    pybind11::class_<PyJoint>(m, "Joint")
        .def(pybind11::init<const std::string &, const double &, const double &, const double &>(), pybind11::arg("name"), pybind11::arg("minAngle")=-1.5708, pybind11::arg("maxAngle")=1.5708, pybind11::arg("maxRate")=6)
        // .def(pybind11::init<const std::string &, const double &, const double &>(), pybind11::arg("name"), pybind11::arg("minAngle"), pybind11::arg("maxAngle"))
        // .def(pybind11::init<const std::string &>(), pybind11::arg("name"))
        .def("setMinAngle", &Joint::setMinAngle, "Set minimum allowed angle of joint (rad)")
        .def("setMaxAngle", &Joint::setMaxAngle, "Set maximum allowed angle of joint (rad)")
        .def("setMaxRate", &Joint::setMaxRate, "Set maximum allowed angular rate of joint (rad/sec)")
        .def_property_readonly("name", &Joint::getName, "The joints's name")
        .def_property_readonly("getMinAngle", &Joint::getMinAngle, "Get minimum allowed angle of this joint (rad)")
        .def_property_readonly("getMaxAngle", &Joint::getMaxAngle, "Get maximum allowed angle of this joint (rad)")
        .def_property_readonly("getMaxRate", &Joint::getMaxRate, "Get maximum allowed angular rate of this joint (rad/sec)")
        .def("__repr__", [](const Joint &joint) {
            return "<Joint named '" + joint.getName() + "'>";
        });
};
