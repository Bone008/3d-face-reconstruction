#pragma once

#include <pcl/common/common.h>

class Sensor {
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	std::vector<Eigen::Vector3f>  m_featurePoints;

    explicit Sensor() : m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {}

};