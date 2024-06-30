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

bool PyLeg::setSegmentLengthsPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setSegmentLengths(map);
}

bool PyLeg::setSegmentOffsetsPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setSegmentOffsets(map);
}

bool PyLeg::setSegmentMinAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setSegmentMinAngles(map);
}

bool PyLeg::setSegmentMaxAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setSegmentMaxAngles(map);
}

bool PyLeg::setSegmentCurrentAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, double> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<double>();
    map[key] = value;
  }

  return setSegmentCurrentAngles(map);
}

pybind11::dict PyLeg::getSegmentLengthsPy()
{
  std::map<std::string, double> map;
  map = getSegmentLengths();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentOffsetsPy()
{
  std::map<std::string, double> map;
  map = getSegmentOffsets();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentMinAnglesPy()
{
  std::map<std::string, double> map;
  map = getSegmentMinAngles();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentMaxAnglesPy()
{
  std::map<std::string, double> map;
  map = getSegmentMaxAngles();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentCurrentAnglesPy()
{
  std::map<std::string, double> map;
  map = getSegmentCurrentAngles();
  
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
        .def(pybind11::init<const std::string &, const std::list<std::string> &, const std::list<Eigen::Vector3d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &, const std::list<Eigen::Vector4d> &,           const int &,              const double &,          const double &>(), 
                          pybind11::arg("name"),        pybind11::arg("legIds"),        pybind11::arg("legOrigins"),        pybind11::arg("legLengths"),      pybind11::arg("segmentOffsets"),         pybind11::arg("segmentMins"),         pybind11::arg("segmentMaxs"),        pybind11::arg("segmentRates"), pybind11::arg("gait"), pybind11::arg("stepLength"), pybind11::arg("stepHeight"))
        .def("setLegPosition", static_cast<std::vector<double> (PyBot::*)(int, const Eigen::Vector3d &)>(&PyBot::setLegPosition), pybind11::arg("index"), pybind11::arg("point"), "Send Point get Angles")
        .def("setLegPosition", static_cast<std::vector<double> (PyBot::*)(const std::string &, const Eigen::Vector3d &)>(&PyBot::setLegPosition), pybind11::arg("id"), pybind11::arg("point"), "Send Point get Angles")
        .def("setLegPositions", static_cast<std::vector<std::vector<double>> (PyBot::*)(const std::vector<Eigen::Vector3d> &)>(&PyBot::setLegPositions), pybind11::arg("points"), "Send Points get Angles")
        .def("setGait", &PyBot::setGait, "Set the bots gait")
        .def("setStepLength", &PyBot::setStepLength, "Set the bots step length")
        .def("setstepHeight", &PyBot::setStepHeight, "Set the bots step height")
        .def_property_readonly("name", &PyBot::getName, "The bot's name")
        .def_property_readonly("gait", &PyBot::getGait, "The bot's current gait")
        .def_property_readonly("stepLength", &PyBot::getStepLength, "The current step length")
        .def_property_readonly("stepHeight", &PyBot::getStepHeight, "The current step height")
        .def("__repr__", [](const PyBot &bot) {
            return "<Bot named '" + bot.getName() + "'>";
        });

    pybind11::class_<PyLeg>(m, "Leg")
        .def(pybind11::init<const std::string &, const Eigen::Vector3d &,  const Eigen::Vector4d &,       const Eigen::Vector4d &,    const Eigen::Vector4d &,    const Eigen::Vector4d &,  const Eigen::Vector4d &>(), 
                          pybind11::arg("name"), pybind11::arg("origin"), pybind11::arg("lengths"), pybind11::arg("segmentOffsets"), pybind11::arg("segmentMins"), pybind11::arg("segmentMaxs"), pybind11::arg("segmentRates"))
        .def("getAnglesFromPoint", &PyLeg::getAnglesFromPointPy, "Send Point get Angles")
        .def("getPointFromAngles", &PyLeg::getPointFromAnglesPy, "Send Angles get Point")
        .def("setSegmentLengths", &PyLeg::setSegmentLengthsPy, "Set distance to next segment (meters)")
        .def("setSegmentOffsets", &PyLeg::setSegmentOffsetsPy, "Set offset of segments from straight when angle equals zero (rad)")
        .def("setSegmentMinAngles", &PyLeg::setSegmentMinAnglesPy, "Set min angles of segments (rad)")
        .def("setSegmentMaxAngles", &PyLeg::setSegmentMaxAnglesPy, "Set max angles of segments (rad)")
        .def("setSegmentCurrentAngles", &PyLeg::setSegmentCurrentAnglesPy, "Set current angles of segments (rad)")
        .def_property_readonly("name", &PyLeg::getName, "The leg's name")
        .def_property_readonly("origin", &PyLeg::getOrigin, "The leg's origin offset (x, y, z, yaw)")
        .def_property_readonly("segments", &PyLeg::getSegments, "The leg's segments (coxa, femur, tibia, tarsus)")
        .def_property_readonly("segmentLengths", &PyLeg::getSegmentLengthsPy, "The distance to the next segment (coxa, femur, tibia, tarsus)")
        .def_property_readonly("segmentOffsets", &PyLeg::getSegmentOffsetsPy, "The leg segment's offset from straight at zero angle (coxa, femur, tibia, tarsus)")
        .def_property_readonly("segmentMinAngles", &PyLeg::getSegmentMinAnglesPy, "Get min angles of all the segments (rad)")
        .def_property_readonly("segmentMaxAngles", &PyLeg::getSegmentMaxAnglesPy, "Get max angles of all the segments (rad)")
        .def_property_readonly("segmentCurrentAngles", &PyLeg::getSegmentCurrentAnglesPy, "Get current angle of all the segments (rad)")
        .def("__repr__", [](const PyLeg &leg) {
            return "<Leg named '" + leg.getName() + "'>";
        });

    pybind11::class_<PySegment>(m, "Segment")
        .def(pybind11::init<std::string &,                double &,                    double &,                          double &,                         double &,                double &>(), 
                    pybind11::arg("name"), pybind11::arg("length"), pybind11::arg("offset")=0.0, pybind11::arg("minAngle")=-1.5708, pybind11::arg("maxAngle")=1.5708, pybind11::arg("maxRate")=6)
        .def("setLength", &PySegment::setLength, "Set distance from this segment to the next segment (meter)")
        .def("setOffset", &PySegment::setOffset, "Set offset of this segment from straight when angle is zero (rad)")
        .def("setMinAngle", &PySegment::setMinAngle, "Set minimum allowed angle of segment (rad)")
        .def("setMaxAngle", &PySegment::setMaxAngle, "Set maximum allowed angle of segment (rad)")
        .def("setMaxRate", &PySegment::setMaxRate, "Set maximum allowed angular rate of segment (rad/sec)")
        .def("setCurrentAngle", &PySegment::setCurrentAngle, "Set current angle of segment (rad)")
        .def_property_readonly("name", &PySegment::getName, "The segments's name")
        .def_property_readonly("length", &PySegment::getLength, "Get distance from this segment to the next segment (meter)")
        .def_property_readonly("offset", &PySegment::getOffset, "Get offset of segment from straight at zero angle (rad)")
        .def_property_readonly("minAngle", &PySegment::getMinAngle, "Get minimum allowed angle of this segment (rad)")
        .def_property_readonly("maxAngle", &PySegment::getMaxAngle, "Get maximum allowed angle of this segment (rad)")
        .def_property_readonly("maxRate", &PySegment::getMaxRate, "Get maximum allowed angular rate of this segment (rad/sec)")
        .def_property_readonly("currentAngle", &PySegment::getCurrentAngle, "Get current angle of this segment (rad)")
        .def("__repr__", [](const PySegment &segment) {
            return "<Segment named '" + segment.getName() + "'>";
        });
};
