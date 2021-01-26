#include "lidar_localization/sensor_data/joint_data.hpp"

#include <iostream>

namespace lidar_localization {
Eigen::Matrix3f JointData::GetRotationMatrix() {
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f ea(this->angle, 0.0, 0.0);  //0 1 2 对应 z y x

  R = Eigen::AngleAxisf(ea[0], Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(ea[1], Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(ea[2], Eigen::Vector3f::UnitX());
  return R;
}

Eigen::Matrix4f JointData::GetTranslationMatrix() {
  Eigen::Matrix3f R = this->GetRotationMatrix();
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3,3>(0,0) = R;
  return T;
}

bool JointData::SyncData(std::deque<JointData>& UnsyncedData, std::deque<JointData>& SyncedData, double sync_time) {
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  if (UnsyncedData.size() < 2)
    return false;

  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time > sync_time)
      return false;
    if (UnsyncedData.at(1).time < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }
    if (sync_time - UnsyncedData.front().time > 0.2) {
      UnsyncedData.pop_front();
      return false;
    }
    if (UnsyncedData.at(1).time - sync_time > 0.2) {
      UnsyncedData.pop_front();
      return false;
    }
    break;
  }

  if (UnsyncedData.size() < 2)
    return true;

  JointData front_data = UnsyncedData.at(0);
  JointData back_data = UnsyncedData.at(1);
  JointData synced_data;

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;
  synced_data.angle = front_data.angle * front_scale + back_data.angle * back_scale;
  SyncedData.push_back(synced_data);

  return true;
}

}