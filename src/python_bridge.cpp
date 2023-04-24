#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>

#include "bot.h"
#include "leg.h"
#include "joint.h"


PYBIND11_MODULE(kinematics, m) 
{
    pybind11::class_<Bot>(m, "Bot")
        .def(pybind11::init<const std::string &, const std::string &, const std::list<std::string> &, const double &>(), pybind11::arg("name"), pybind11::arg("frame_id"), pybind11::arg("legIds"), pybind11::arg("wait_for_tf_delay") = 10)
        .def_property_readonly("name", &Bot::getName, "The bot's name")
        .def("__repr__", [](const Bot &bot) {
            return "<Bot named '" + bot.getName() + "'>";
        });

    pybind11::class_<Leg>(m, "Leg")
        .def(pybind11::init<const std::string &, const Eigen::Vector4d &, const Eigen::Vector4d &>(), pybind11::arg("name"), pybind11::arg("origin"), pybind11::arg("lengths"))
        .def("getAnglesFromPoint", &Leg::getAnglesFromPointPy, "Send Point get Angles")
        .def("getPointFromAngles", &Leg::getPointFromAnglesPy, "Send Angles get Point")
        .def_property_readonly("name", &Leg::getName, "The leg's name")
        .def_property_readonly("origin", &Leg::getOrigin, "The leg's origin offset (x, y, z, yaw)")
        .def_property_readonly("lengths", &Leg::getLengths, "The leg's lengths (coxa, femur, tibia, tarsus)")
        .def("__repr__", [](const Leg &leg) {
            return "<Leg named '" + leg.getName() + "'>";
        });

    pybind11::class_<Joint>(m, "Joint")
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
}

