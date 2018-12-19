#pragma once
#include <Eigen/Eigen>
#include "FaceModel.h"
#include <pcl/common/common.h>

// returns pose
Eigen::Matrix4f computeCoarseAlignment(const FaceModel& model, const Sensor& inputSensor);