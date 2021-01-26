/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 13:10:51
 */
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

void GNSSSubscriber::msg_callback(const lidar_localization::RTKData::ConstPtr &rtk_ptr) {
  buff_mutex_.lock();
  GNSSData gnss_data;
//    gnss_data.time = rtk_ptr->header.stamp.toSec();
  gnss_data.time = 0.0;
  gnss_data.latitude = rtk_ptr->latitude;
  gnss_data.longitude = rtk_ptr->longitude;
  gnss_data.altitude = rtk_ptr->height;
  gnss_data.heading = rtk_ptr->heading;

  new_gnss_data_.push_back(gnss_data);
  buff_mutex_.unlock();
}

void GNSSSubscriber::ParseData(std::deque<GNSSData> &gnss_data_buff) {
  buff_mutex_.lock();
  if (new_gnss_data_.size() > 0) {
    gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
    new_gnss_data_.clear();
  }
  buff_mutex_.unlock();
}
}