#pragma once
class FaceModel;

Eigen::VectorXf initializeShapeParameters(const FaceModel& model, const Eigen::Matrix4f& pose, pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr inputCloud);
