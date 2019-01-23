#pragma once

#include <pcl/common/common.h>
#include <pcl/Vertices.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf& points);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf& points, const Eigen::Matrix4Xi& vertexColors);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointsToCloud(const Eigen::VectorXf& points, const Eigen::Matrix3Xf& normals);

std::vector<pcl::Vertices> trianglesToVertexList(const Eigen::Matrix3Xi& triangles);