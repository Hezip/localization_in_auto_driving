/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh, std::string cloud_topic) {
  // subscriber
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/livox/lidar_all", 100000);
  joint_sub_ptr_ = std::make_shared<JointSubscriber>(nh, "/joint_states", 100000);
//    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
//    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
  gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/rtk_data", 1000000);
//    joint_sub_ptr_ = std::make_shared<>();
  init_to_map_ptr_ = std::make_shared<TFListener>(nh, "/map", "/base_link");
  lidar_to_cabin_ptr_ = std::make_shared<TFListener>(nh, "/cabin_link", "/livox_frame");
  lidar_to_base_ptr_ = std::make_shared<TFListener>(nh, "/base_link", "/livox_frame");
  // publisher
  cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/livox_frame", 100);
  joint_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_joint", "/base_link", "/livox_frame", 100);
  gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/odom", "/base_link", 100);

//    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
  if (!ReadData())
    return false;

  if (!InitCalibration())
    return false;

  if (!InitGNSS())
    return false;

  while (HasData()) {
    if (!ValidData())
      continue;

//    TransformDatas();

    PublishData();
  }

  return true;
}

bool DataPretreatFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);

  static std::deque<GNSSData> unsynced_gnss_;
  static std::deque<JointData> unsynced_joint_;

  gnss_sub_ptr_->ParseData(gnss_data_buff_);
  joint_sub_ptr_->ParseData(unsynced_joint_);

  if (cloud_data_buff_.size() == 0)
    return false;

  double cloud_time = cloud_data_buff_.front().time;
  bool valid_joint = JointData::SyncData(unsynced_joint_, joint_data_buff_, cloud_time);
  bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

  if (!valid_joint) {
    cloud_data_buff_.pop_front();
    ROS_ERROR("data unstable");
    return false;
  }

  return true;
}

bool DataPretreatFlow::InitCalibration() {
  static bool calibration_received = false;
  if (!calibration_received) {
    if (lidar_to_cabin_ptr_->LookupData(lidar_to_cabin_)) {
      calibration_received = true;
    }
  }
  return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
  static bool gnss_inited = false;
  if (!gnss_inited && !gnss_data_buff_.empty()) {
    GNSSData gnss_data = gnss_data_buff_.front();
    gnss_data.InitOriginPosition();
    gnss_inited = true;
  }

  return gnss_inited;
}

bool DataPretreatFlow::HasData() {
  if (cloud_data_buff_.size() == 0)
    return false;

  if (joint_data_buff_.size() == 0)
    return false;

  if (gnss_data_buff_.size() == 0)
    return false;

  return true;
}

bool DataPretreatFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();
  current_joint_data_ = joint_data_buff_.front();
//
//    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
  double diff_joint_time = current_cloud_data_.time - current_joint_data_.time;
//    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
//        cloud_data_buff_.pop_front();
//        return false;
//    }
  if (diff_joint_time < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_joint_time > 0.05) {
    joint_data_buff_.pop_front();
    return false;
  }
//
//    if (diff_gnss_time > 0.05) {
//        gnss_data_buff_.pop_front();
//        return false;
//    }
//
  cloud_data_buff_.pop_front();
  joint_data_buff_.pop_front();
//  gnss_data_buff_.pop_front();
  gnss_data_buff_.clear();

  return true;
}

bool DataPretreatFlow::TransformDatas() {
  gnss_pose_ = Eigen::Matrix4f::Identity();

  current_gnss_data_.UpdateXYZ();
  gnss_pose_(0, 3) = current_gnss_data_.local_E;
  gnss_pose_(1, 3) = current_gnss_data_.local_N;
  gnss_pose_(2, 3) = current_gnss_data_.local_U;

  gnss_pose_.block<3, 3>(0, 0) = current_gnss_data_.GetRotationMatrix();
//  gnss_pose_ *= lidar_to_imu_;
  return true;
}

bool DataPretreatFlow::PublishData() {

  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);

  lidar_to_base_ = Eigen::Matrix4f::Identity();
  lidar_to_base_ = current_joint_data_.GetTranslationMatrix() * lidar_to_cabin_;
  lidar_to_base_(2, 3) = 3.0538;
  joint_pub_ptr_->Publish(lidar_to_base_, current_joint_data_.time);

//  gnss_pub_ptr_->Publish(gnss_pose_);

  return true;
}
}