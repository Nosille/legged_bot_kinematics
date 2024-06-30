#include <cmath>
#include <vector>
#include <iterator>
#include <algorithm>

#include "segment.h"

// Constructors
Segment::Segment(std::string _name,
             double _length,
             double _offset = 0.0,
             double _minAngle = -M_PI,
             double _maxAngle = M_PI,
             double _maxRate = 6.0
            )
            : name_(_name)
{
  length_ = _length;
  offset_ = _offset;
  minAngle_ = std::min(_minAngle, _maxAngle);
  maxAngle_ = std::max(_minAngle, _maxAngle);
  maxRate_  = _maxRate;
  currentAngle_ = 0.0;
}

Segment::Segment(std::string _name, const Segment &_oldSegment) 
{
  name_ = _oldSegment.getName();

  length_ = _oldSegment.getLength();
  offset_ = _oldSegment.getOffset();
  minAngle_ = _oldSegment.getMinAngle();
  maxAngle_ = _oldSegment.getMaxAngle();
  maxRate_  = _oldSegment.getMaxRate();
  currentAngle_ = _oldSegment.getCurrentAngle();
}

// Methods
std::string Segment::getName() const 
{ 
  return name_; 
}

double Segment::getLength() const 
{ 
  return length_; 
}

double Segment::getOffset() const 
{ 
  return offset_; 
}

double Segment::getMinAngle() const
{ 
  return minAngle_; 
}

double Segment::getMaxAngle() const
{ 
  return maxAngle_; 
}

double Segment::getMaxRate() const
{ 
  return maxRate_; 
}

double Segment::getCurrentAngle() const
{ 
  return currentAngle_; 
}

bool Segment::setLength(double _length) 
{ 
  if(_length <= 0.0) return false;

  length_ = _length; 
  return true;  
}

bool Segment::setOffset(double _offset) 
{ 
  if(_offset < -M_PI) return false;
  if(_offset > +M_PI) return false;
  
  offset_ = _offset; 
  return true;  
}

bool Segment::setMinAngle(double _minAngle) 
{ 
  if(_minAngle < -M_PI) return false;
  if(_minAngle > maxAngle_) return false;

  minAngle_ = _minAngle; 
  return true;  
}

bool Segment::setMaxAngle(double _maxAngle) 
{ 
  if(_maxAngle > +M_PI) return false;
  if(_maxAngle < minAngle_) return false;

  maxAngle_ = _maxAngle; 
  return true;
}

bool Segment::setMaxRate(double _maxRate) 
{ 
  maxRate_ = _maxRate; 
  return true;
}

bool Segment::setCurrentAngle(double _currentAngle) 
{ 
  currentAngle_ = _currentAngle;

  if(_currentAngle < minAngle_) return false;
  if(_currentAngle > maxAngle_) return false;
  
  return true;
}
