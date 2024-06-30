#include "python_bridge.h"

pybind11::dict PyLeg::calcAnglesFromPointPy(const Eigen::Vector3f &_point)
{
  std::map<std::string, float> map;
  map = calcAnglesFromPoint(_point);
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

Eigen::Vector3f PyLeg::calcPointFromAnglesPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string,float> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<float>();
    map[key] = value;
  }

  // Get point
  return calcPointFromAngles(map);
}

bool PyLeg::setSegmentLengthsPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, float> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<float>();
    map[key] = value;
  }

  return setSegmentLengths(map);
}

bool PyLeg::setSegmentAnglesOffsetPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, float> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<float>();
    map[key] = value;
  }

  return setSegmentAnglesOffset(map);
}

bool PyLeg::setSegmentAnglesMinPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, float> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<float>();
    map[key] = value;
  }

  return setSegmentAnglesMin(map);
}

bool PyLeg::setSegmentAnglesMaxPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, float> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<float>();
    map[key] = value;
  }

  return setSegmentAnglesMax(map);
}

bool PyLeg::setSegmentAnglesCurrentPy(const pybind11::dict &_dict)
{
  // Convert dict to map
  std::map<std::string, float> map;
  for (auto item : _dict)
  {
    auto key = item.first.cast<std::string>();
    auto value = item.second.cast<float>();
    map[key] = value;
  }

  return setSegmentAnglesCurrent(map);
}

pybind11::dict PyLeg::getSegmentLengthsPy()
{
  std::map<std::string, float> map;
  map = getSegmentLengths();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentAnglesOffsetPy()
{
  std::map<std::string, float> map;
  map = getSegmentAnglesOffset();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentAnglesMinPy()
{
  std::map<std::string, float> map;
  map = getSegmentAnglesMin();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentAnglesMaxPy()
{
  std::map<std::string, float> map;
  map = getSegmentAnglesMax();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

pybind11::dict PyLeg::getSegmentAnglesCurrentPy()
{
  std::map<std::string, float> map;
  map = getSegmentAnglesCurrent();
  
  pybind11::dict dict;

  for (auto item : map)
  {
    dict[pybind11::cast(item.first)] = item.second;
  }

  return dict;
}

PYBIND11_MODULE(leg_kinematics, m) 
{
    pybind11::class_<PyBot>(m, "LeggedBot")
        .def(pybind11::init<const std::string &, const std::list<std::string> &, const std::list<Eigen::VectorXf> &, const std::list<Eigen::Vector4f> &, const std::list<Eigen::Vector4f> &, const std::list<Eigen::Vector4f> &, const std::list<Eigen::Vector4f> &, const std::list<Eigen::Vector4f> &,           const int &,              const float &,          const float &>(), 
                          pybind11::arg("name"),        pybind11::arg("legIds"),        pybind11::arg("legOrigins"),        pybind11::arg("legLengths"),    pybind11::arg("segmentOffsets"),       pybind11::arg("segmentMins"),       pybind11::arg("segmentMaxs"),      pybind11::arg("segmentRates"), pybind11::arg("gait"), pybind11::arg("stepLength"), pybind11::arg("stepHeight"))
        .def("calcLegPosition", static_cast<std::vector<float> (PyBot::*)(uint8_t, const Eigen::Vector3f &)>(&PyBot::calcLegPosition), pybind11::arg("index"), pybind11::arg("point"), "Send Point get Angles")
        .def("calcLegPosition", static_cast<std::vector<float> (PyBot::*)(const std::string &, const Eigen::Vector3f &)>(&PyBot::calcLegPosition), pybind11::arg("id"), pybind11::arg("point"), "Send Point get Angles")
        .def("calcLegPositions", static_cast<std::vector<std::vector<float>> (PyBot::*)(const std::vector<Eigen::Vector3f> &)>(&PyBot::calcLegPositions), pybind11::arg("points"), "Send Points get Angles")
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
        .def(pybind11::init<const std::string &, const Eigen::Vector3f &,  const Eigen::Vector4f &,       const Eigen::Vector4f &,    const Eigen::Vector4f &,    const Eigen::Vector4f &,  const Eigen::Vector4f &>(), 
                          pybind11::arg("name"), pybind11::arg("origin"), pybind11::arg("segmentLengths"), pybind11::arg("segmentOffsets"), pybind11::arg("segmentMins"), pybind11::arg("segmentMaxs"), pybind11::arg("segmentRates"))
        .def("getAnglesFromPoint", &PyLeg::calcAnglesFromPointPy, "Send Point get Angles")
        .def("getPointFromAngles", &PyLeg::calcPointFromAnglesPy, "Send Angles get Point")
        .def("setSegmentLengths", &PyLeg::setSegmentLengthsPy, "Set distance to next segment (meters)")
        .def("setSegmentAnglesOffset", &PyLeg::setSegmentAnglesOffsetPy, "Set offset of segments from straight when angle equals zero (rad)")
        .def("setSegmentAnglesMin", &PyLeg::setSegmentAnglesMinPy, "Set min angles of segments (rad)")
        .def("setSegmentAnglesMax", &PyLeg::setSegmentAnglesMaxPy, "Set max angles of segments (rad)")
        .def("setSegmentAnglesCurrent", &PyLeg::setSegmentAnglesCurrentPy, "Set current angles of segments (rad)")
        .def_property_readonly("name", &PyLeg::getName, "The leg's name")
        .def_property_readonly("origin", &PyLeg::getOrigin, "The leg's origin offset (x, y, z, yaw)")
        .def_property_readonly("segments", &PyLeg::getSegments, "The leg's segments (coxa, femur, tibia, tarsus)")
        .def_property_readonly("segmentLengths", &PyLeg::getSegmentLengthsPy, "The distance to the next segment (coxa, femur, tibia, tarsus)")
        .def_property_readonly("segmentAnglesOffset", &PyLeg::getSegmentAnglesOffsetPy, "The leg segment's offset from straight at zero angle (coxa, femur, tibia, tarsus)")
        .def_property_readonly("segmentAnglesMin", &PyLeg::getSegmentAnglesMinPy, "Get min angles of all the segments (rad)")
        .def_property_readonly("segmentAnglesMax", &PyLeg::getSegmentAnglesMaxPy, "Get max angles of all the segments (rad)")
        .def_property_readonly("segmentAnglesCurrent", &PyLeg::getSegmentAnglesCurrentPy, "Get current angle of all the segments (rad)")
        .def("__repr__", [](const PyLeg &leg) {
            return "<Leg named '" + leg.getName() + "'>";
        });

    pybind11::class_<PySegment>(m, "Segment")
        .def(pybind11::init<std::string &,                float &,                    float &,                          float &,                         float &,                float &>(), 
                    pybind11::arg("name"), pybind11::arg("length"), pybind11::arg("offset")=0.0, pybind11::arg("minAngle")=-1.5708, pybind11::arg("maxAngle")=1.5708, pybind11::arg("maxRate")=6)
        .def("setLength", &PySegment::setLength, "Set distance from this segment to the next segment (meter)")
        .def("setOffset", &PySegment::setAngleOffset, "Set offset of this segment from straight when angle is zero (rad)")
        .def("setAngleMin", &PySegment::setAngleMin, "Set minimum allowed angle of segment (rad)")
        .def("setAngleMax", &PySegment::setAngleMax, "Set maximum allowed angle of segment (rad)")
        .def("setAngleRateMax", &PySegment::setAngleRateMax, "Set maximum allowed angular rate of segment (rad/sec)")
        .def("setAngleCurrent", &PySegment::setAngleCurrent, "Set current angle of segment (rad)")
        .def_property_readonly("name", &PySegment::getName, "The segments's name")
        .def_property_readonly("length", &PySegment::getLength, "Get distance from this segment to the next segment (meter)")
        .def_property_readonly("angleOffset", &PySegment::getAngleOffset, "Get offset of segment from straight at zero angle (rad)")
        .def_property_readonly("angleMin", &PySegment::getAngleMin, "Get minimum allowed angle of this segment (rad)")
        .def_property_readonly("angleMax", &PySegment::getAngleMax, "Get maximum allowed angle of this segment (rad)")
        .def_property_readonly("angleRateMax", &PySegment::getAngleRateMax, "Get maximum allowed angular rate of this segment (rad/sec)")
        .def_property_readonly("angleCurrent", &PySegment::getAngleCurrent, "Get current angle of this segment (rad)")
        .def("__repr__", [](const PySegment &segment) {
            return "<Segment named '" + segment.getName() + "'>";
        });
};
