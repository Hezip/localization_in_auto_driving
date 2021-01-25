#include "lidar_localization/subscriber/joint_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization{
JointSubscriber::JointSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &JointSubscriber::msg_callback, this);
}

void JointSubscriber::msg_callback(const sensor_msgs::JointStateConstPtr& joint_msg_ptr) {
  buff_mutex_.lock();
  JointData joint_data;
  joint_data.time = joint_msg_ptr->header.stamp.toSec();

  //set the position
  if (joint_msg_ptr->name[0] != "cabin_joint") {
    ROS_ERROR("joint name is not correct");
    buff_mutex_.unlock();
    return;
  }

  joint_data.angle = joint_msg_ptr->position[0];

  new_joint_data_.push_back(joint_data);
  buff_mutex_.unlock();
}

void JointSubscriber::ParseData(std::deque<JointData>& joint_data_buff) {
  buff_mutex_.lock();
  if (new_joint_data_.size() > 0) {
    joint_data_buff.insert(joint_data_buff.end(), new_joint_data_.begin(), new_joint_data_.end());
    new_joint_data_.clear();
  }
  buff_mutex_.unlock();
}
}