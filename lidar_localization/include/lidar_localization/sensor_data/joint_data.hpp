/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_JOINT_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_JOINT_DATA_HPP_

#include <Eigen/Dense>
#include <deque>

namespace lidar_localization {
class JointData {
 public:

  double angle = 0.0;
  double time = 0.0;

 public:
  Eigen::Matrix3f GetRotationMatrix();
  Eigen::Matrix4f GetTranslationMatrix();
  static bool SyncData(std::deque<JointData>& UnsyncedData, std::deque<JointData>& SyncedData, double sync_time);
};
}

#endif