#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>

#include "bot.h"
#include "leg.h"
#include "joint.h"

class PyBot : public Bot
{
  public:
    using Bot::Bot;
};

class PyLeg : public Leg
{
  public:
    using Leg::Leg;

    pybind11::dict getAnglesFromPointPy(const Eigen::Vector3d &_point);
    Eigen::Vector3d getPointFromAnglesPy(const pybind11::dict &_dict);

    bool setJointLengthsPy(const pybind11::dict &_dict);
    bool setJointOffsetsPy(const pybind11::dict &_dict);
    bool setJointMinAnglesPy(const pybind11::dict &_dict);
    bool setJointMaxAnglesPy(const pybind11::dict &_dict);
    bool setJointCurrentAnglesPy(const pybind11::dict &_dict);
    
    pybind11::dict getJointLengthsPy();
    pybind11::dict getJointOffsetsPy();
    pybind11::dict getJointMinAnglesPy();
    pybind11::dict getJointMaxAnglesPy();
    pybind11::dict getJointCurrentAnglesPy();
};

class PyJoint : public Joint
{
  public:
    using Joint::Joint;
};
