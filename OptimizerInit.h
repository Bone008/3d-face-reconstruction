#pragma once
#include "FaceModel.h"
#include "Sensor.h"

Eigen::VectorXf initializeShapeParameters(const FaceModel& model, const Eigen::Matrix4f& pose, pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr inputCloud);
