#pragma once
#include <Eigen/Eigen>
#include "FaceModel.h"
#include <pcl/common/common.h>

// returns pose
Eigen::Matrix4f compute_coarse_alignment(FaceModel& model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);