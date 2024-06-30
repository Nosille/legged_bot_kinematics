#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>

#include "bot.h"
#include "leg.h"
#include "segment.h"

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

    bool setSegmentLengthsPy(const pybind11::dict &_dict);
    bool setSegmentOffsetsPy(const pybind11::dict &_dict);
    bool setSegmentMinAnglesPy(const pybind11::dict &_dict);
    bool setSegmentMaxAnglesPy(const pybind11::dict &_dict);
    bool setSegmentCurrentAnglesPy(const pybind11::dict &_dict);
    
    pybind11::dict getSegmentLengthsPy();
    pybind11::dict getSegmentOffsetsPy();
    pybind11::dict getSegmentMinAnglesPy();
    pybind11::dict getSegmentMaxAnglesPy();
    pybind11::dict getSegmentCurrentAnglesPy();
};

class PySegment : public Segment
{
  public:
    using Segment::Segment;
};
