#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>

#include "legged_bot.h"
#include "leg.h"
#include "segment.h"

class PyBot : public LeggedBot
{
  public:
    using LeggedBot::LeggedBot;
};

class PyLeg : public Leg
{
  public:
    using Leg::Leg;

    pybind11::dict calcAnglesFromPointPy(const Eigen::Vector3f &_point);
    Eigen::Vector3f calcPointFromAnglesPy(const pybind11::dict &_dict);

    bool setSegmentLengthsPy(const pybind11::dict &_dict);
    bool setSegmentAnglesOffsetPy(const pybind11::dict &_dict);
    bool setSegmentAnglesMinPy(const pybind11::dict &_dict);
    bool setSegmentAnglesMaxPy(const pybind11::dict &_dict);
    bool setSegmentAnglesCurrentPy(const pybind11::dict &_dict);
    
    pybind11::dict getSegmentLengthsPy();
    pybind11::dict getSegmentAnglesOffsetPy();
    pybind11::dict getSegmentAnglesMinPy();
    pybind11::dict getSegmentAnglesMaxPy();
    pybind11::dict getSegmentAnglesCurrentPy();
};

class PySegment : public Segment
{
  public:
    using Segment::Segment;
};
