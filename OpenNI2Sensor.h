#pragma once
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "Sensor.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr PointCloudT;

class OpenNI2Sensor : public Sensor {
public:

    OpenNI2Sensor();

    void cloud_cb (const PointCloudT &cloud);
private:
    pcl::visualization::CloudViewer m_viewer;
    PointCloudT m_cloud_rgba;
};