#pragma once
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "Sensor.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr PointCloudT;

class OpenNI2Sensor : public Sensor {
public:

    OpenNI2Sensor(const std::string& filenamePcd, const std::string& filenameFeaturePoints);

private:
    void capturePointCloudToFile(const std::string& filenamePcd);
    void loadFromFile(const std::string& filenamePcd);
};