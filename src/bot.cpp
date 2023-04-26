#include "bot.h"

Bot::Bot(const std::string &_name, const std::string &_FrameId, const std::list<std::string> &_legIds, const double _wait_for_tf_delay = 10)
: name_(_name)
, nh_()
{
  //TF Listener
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  frame_id_ = _FrameId;
  wait_for_tf_delay_ = _wait_for_tf_delay;
  
  for(std::string legId : _legIds)
  {
    // Create frame names
    std::string bodyFrame = name_ + "/" + frame_id_;
    std::string centerFrame = name_ + "/leg_center_" + legId;
    std::string coxaFrame = name_ + "/coxa_" + legId;
    std::string femurFrame = name_ + "/femur_" + legId;
    std::string tibiaFrame = name_ + "/tibia_" + legId;
    std::string tarsusFrame = name_ + "/tarsus_" + legId;
    std::string endFrame = name_ + "/end_" + legId;

    // Find transforms
    geometry_msgs::TransformStamped center, coxa, femur, tibia, tarsus;
    assert(getTransform(center, coxaFrame, bodyFrame, ros::Time(), ros::Duration(wait_for_tf_delay_)));
    assert(getTransform(coxa, femurFrame, coxaFrame, ros::Time(), ros::Duration(0.0)));
    assert(getTransform(femur, tibiaFrame, femurFrame, ros::Time(), ros::Duration(0.0)));
    if(!getTransform(tibia, tarsusFrame, tibiaFrame, ros::Time(), ros::Duration(0.0)))
    {
      assert(getTransform(tibia, endFrame, tibiaFrame, ros::Time(), ros::Duration(0.0)));
    }
    else
      assert(getTransform(tarsus, endFrame, tarsusFrame, ros::Time(), ros::Duration(0.0)));

    // Get origin
    Eigen::Vector4d origin;
    tf2::Quaternion q;
    tf2::convert(center.transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    origin[0] = center.transform.translation.x;
    origin[1] = center.transform.translation.y;
    origin[2] = center.transform.translation.z;
    origin[3] = yaw;

    // Calculate Lengths
    double coxaLength = sqrt(coxa.transform.translation.x * coxa.transform.translation.x
                           + coxa.transform.translation.y * coxa.transform.translation.y
                           + coxa.transform.translation.z * coxa.transform.translation.z);
    double femurLength = sqrt(femur.transform.translation.x * femur.transform.translation.x
                            + femur.transform.translation.y * femur.transform.translation.y
                            + femur.transform.translation.z * femur.transform.translation.z);
    double tibiaLength = sqrt(tibia.transform.translation.x * tibia.transform.translation.x
                            + tibia.transform.translation.y * tibia.transform.translation.y
                            + tibia.transform.translation.z * tibia.transform.translation.z);
    double tarsusLength = sqrt(tarsus.transform.translation.x * tarsus.transform.translation.x
                             + tarsus.transform.translation.y * tarsus.transform.translation.y
                             + tarsus.transform.translation.z * tarsus.transform.translation.z);

    ROS_INFO_STREAM("coxaLength: " << coxaLength);
    ROS_INFO_STREAM("femurLength: " << femurLength);
    ROS_INFO_STREAM("tibiaLength: " << tibiaLength);
    ROS_INFO_STREAM("tarsusLength: " << tarsusLength);

    Eigen::Vector4d lengths;
    lengths[0] = coxaLength;
    lengths[1] = femurLength;
    lengths[2] = tibiaLength;
    lengths[3] = tarsusLength;

    Leg leg(legId, origin, lengths);
    legs_.push_back(leg);
  }
}

std::vector<double> Bot::set_leg_position(int _index, Eigen::Vector3d &_point)
{
  std::vector<double> angles;
  legs_[_index-1].getAnglesFromPoint(_point, angles);
  return angles;
}

bool Bot::getTransform(geometry_msgs::TransformStamped &_transform, const std::string &_target_frame, const std::string &_source_frame, const ros::Time &_time, const ros::Duration &_wait_for_tf_delay)
{
  try
  {
    if(tfBuffer_->canTransform(_target_frame, _source_frame, _time, _wait_for_tf_delay))
    {
      _transform = tfBuffer_->lookupTransform(_target_frame, _source_frame, _time);
    }
    else
    {
      // ROS_WARN_STREAM("Failed to get transform " << _source_frame << " to " << _target_frame << " within " << wait_for_tf_delay_ << "!");
      return false;
    }
  }
  catch (tf2::TransformException ex){
    ROS_WARN_STREAM("get Transform: " << ex.what());
    return false;
  }

  return true;
}

/// @brief Get name of Bot
/// @return name of bot
std::string Bot::getName() const { return name_; }
