#include "lidar_localization/sensor_data/joint_data.hpp"

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

}