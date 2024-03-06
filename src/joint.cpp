#include <cmath>
#include <vector>
#include <iterator>
#include <algorithm>

#include "joint.h"

// Constructors
Joint::Joint(std::string _name,
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

Joint::Joint(std::string _name, const Joint &_oldJoint) 
{
  name_ = _oldJoint.getName();

  length_ = _oldJoint.getLength();
  offset_ = _oldJoint.getOffset();
  minAngle_ = _oldJoint.getMinAngle();
  maxAngle_ = _oldJoint.getMaxAngle();
  maxRate_  = _oldJoint.getMaxRate();
  currentAngle_ = _oldJoint.getCurrentAngle();
}

// Methods
std::string Joint::getName() const 
{ 
  return name_; 
}

double Joint::getLength() const 
{ 
  return length_; 
}

double Joint::getOffset() const 
{ 
  return offset_; 
}

double Joint::getMinAngle() const
{ 
  return minAngle_; 
}

double Joint::getMaxAngle() const
{ 
  return maxAngle_; 
}

double Joint::getMaxRate() const
{ 
  return maxRate_; 
}

double Joint::getCurrentAngle() const
{ 
  return currentAngle_; 
}

bool Joint::setLength(double _length) 
{ 
  if(_length <= 0.0) return false;

  length_ = _length; 
  return true;  
}

bool Joint::setOffset(double _offset) 
{ 
  if(_offset < -M_PI) return false;
  if(_offset > +M_PI) return false;
  
  offset_ = _offset; 
  return true;  
}

bool Joint::setMinAngle(double _minAngle) 
{ 
  if(_minAngle < -M_PI) return false;
  if(_minAngle > maxAngle_) return false;

  minAngle_ = _minAngle; 
  return true;  
}

bool Joint::setMaxAngle(double _maxAngle) 
{ 
  if(_maxAngle > +M_PI) return false;
  if(_maxAngle < minAngle_) return false;

  maxAngle_ = _maxAngle; 
  return true;
}

bool Joint::setMaxRate(double _maxRate) 
{ 
  maxRate_ = _maxRate; 
  return true;
}

bool Joint::setCurrentAngle(double _currentAngle) 
{ 
  currentAngle_ = _currentAngle;

  if(_currentAngle < minAngle_) return false;
  if(_currentAngle > maxAngle_) return false;
  
  return true;
}
