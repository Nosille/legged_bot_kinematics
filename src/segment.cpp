#include <cmath>

#include "segment.h"

// Constructors
Segment::Segment(std::string _name,
             float _length,
             float _angleOffset = 0.0,
             float _angleMin = -M_PI,
             float _angleMax = M_PI,
             float _angleRateMax = 6.0,
             uint16_t _anglePrecisionFactor,
             uint16_t _lengthPrecisionFactor
            )
            : name_(_name)
            , anglePrecisionFactor_(_anglePrecisionFactor)
            , lengthPrecisionFactor_(_anglePrecisionFactor)
{
  length_ = _length * lengthPrecisionFactor_;
  angleOffset_ = _angleOffset * anglePrecisionFactor_;
  angleCurrent_ = 0.0;
  angleMin_ = (std::min(_angleMin, _angleMax)) * anglePrecisionFactor_;
  angleMax_ = (std::max(_angleMax, _angleMax)) * anglePrecisionFactor_;
  angleRateMax_  = _angleRateMax  * anglePrecisionFactor_;
}

Segment::Segment(std::string _name, const Segment &_oldSegment) 
{
  name_ = _oldSegment.getName();

  length_ = _oldSegment.length_;
  angleOffset_ = _oldSegment.angleOffset_;
  angleCurrent_ = _oldSegment.angleCurrent_;
  angleMin_ = _oldSegment.angleMin_;
  angleMax_ = _oldSegment.angleMax_;
  angleRateMax_  = _oldSegment.angleRateMax_;

  anglePrecisionFactor_ = _oldSegment.anglePrecisionFactor_;
  lengthPrecisionFactor_= _oldSegment.lengthPrecisionFactor_;
}

// Methods
std::string Segment::getName() const 
{ 
  return name_; 
}

float Segment::getLength() const 
{ 
  return (float)length_ / lengthPrecisionFactor_; 
}

uint32_t Segment::getLengthAsInt() const
{ 
  return length_; 
}

float Segment::getAngleOffset() const 
{ 
  return (float)angleOffset_ / anglePrecisionFactor_; 
}

int32_t Segment::getAngleOffsetAsInt() const
{ 
  return angleOffset_; 
}

float Segment::getAngleCurrent() const
{ 
  return (float)angleCurrent_ / anglePrecisionFactor_; 
}

int32_t Segment::getAngleCurrentAsInt() const
{ 
  return angleCurrent_;
}

float Segment::getAngleMin() const
{ 
  return (float)angleMin_ / anglePrecisionFactor_; 
}

int32_t Segment::getAngleMinAsInt() const
{ 
  return angleMin_;
}

float Segment::getAngleMax() const
{ 
  return (float)angleMax_ / anglePrecisionFactor_; 
}

int32_t Segment::getAngleMaxAsInt() const
{ 
  return angleMax_;
}

float Segment::getAngleRateMax() const
{ 
  return (float)angleRateMax_ / anglePrecisionFactor_; 
}

uint32_t Segment::getAngleRateMaxAsInt() const
{ 
  return angleRateMax_; 
}

uint16_t Segment::getAnglePrecisionFactor() const
{
  return anglePrecisionFactor_;
}

uint16_t Segment::getLengthPrecisionFactor() const
{
  return lengthPrecisionFactor_;
}

bool Segment::setLength(float _length) 
{ 
  if(_length <= 0.0) return false;

  length_ = _length * lengthPrecisionFactor_; 
  return true;  
}

bool Segment::setAngleOffset(float _angleOffset) 
{ 
  if(_angleOffset < -M_PI) return false;
  if(_angleOffset > +M_PI) return false;
  
  angleOffset_ = _angleOffset * anglePrecisionFactor_; 
  return true;  
}

bool Segment::setAngleCurrent(float _angleCurrent) 
{ 
  if(_angleCurrent < angleMin_) return false;
  if(_angleCurrent > angleMax_) return false;

  angleCurrent_ = _angleCurrent * anglePrecisionFactor_;
  return true;
}

bool Segment::setAngleMin(float _angleMin) 
{ 
  if(_angleMin < -M_PI) return false;
  if(_angleMin > angleMax_) return false;

  angleMin_ = _angleMin * anglePrecisionFactor_; 
  return true;  
}

bool Segment::setAngleMax(float _angleMax) 
{ 
  if(_angleMax > +M_PI) return false;
  if(_angleMax < angleMin_) return false;

  angleMax_ = _angleMax * anglePrecisionFactor_; 
  return true;
}

bool Segment::setAngleRateMax(float _angleRateMax) 
{ 
  angleRateMax_ = _angleRateMax * anglePrecisionFactor_; 
  return true;
}

bool Segment::setAnglePrecisionFactor(uint16_t _factor) 
{ 
  anglePrecisionFactor_ = _factor; 
  return true;
}

bool Segment::setLengthPrecisionFactor(uint16_t _factor) 
{ 
  lengthPrecisionFactor_ = _factor; 
  return true;
}