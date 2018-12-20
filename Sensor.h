#pragma once

class Sensor {
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	std::vector<Eigen::Vector3f>  m_featurePoints;
	Eigen::Matrix3f m_cameraIntrinsics;

    explicit Sensor() : m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {}

};
