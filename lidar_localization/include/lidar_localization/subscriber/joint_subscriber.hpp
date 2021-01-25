#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_JOINT_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_JOINT_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "lidar_localization/sensor_data/joint_data.hpp"

namespace lidar_localization {
class JointSubscriber {
 public:
  JointSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  JointSubscriber() = default;
  void ParseData(std::deque<JointData>& deque_pose_data);

 private:
  void msg_callback(const sensor_msgs::JointStateConstPtr& joint_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<JointData> new_joint_data_;

  std::mutex buff_mutex_;
};
}
#endif