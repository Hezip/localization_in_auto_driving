#include <pcl/common/transforms.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include "glog/logging.h"

namespace lidar_localization {

void CloudData::TransformCoordinate(Eigen::Matrix4f transform_matrix) {
  pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, transform_matrix);
}

} //end namespace