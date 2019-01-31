#pragma once

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf& points);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf& points, const Eigen::Matrix4Xi& vertexColors);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointsToCloud(const Eigen::VectorXf& points, const Eigen::Matrix3Xf& normals);

std::vector<pcl::Vertices> trianglesToVertexList(const Eigen::Matrix3Xi& triangles);

// Saves a BMP image using the colors provided by a callback (x, y) --> RGB.
void saveBitmap(const char* filename, int width, int height, std::function<Eigen::Vector3i(unsigned int x, unsigned int y)> pixelColorProvider);
// Saves a BMP image using the colors provided by a callback (x, y) --> RGBA.
void saveBitmapAlpha(const char* filename, int width, int height, std::function<Eigen::Vector4i(unsigned int x, unsigned int y)> pixelColorProvider);
