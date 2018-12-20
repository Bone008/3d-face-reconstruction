#pragma once
#include "Sensor.h"
#include "FeaturePointExtractor.h"
#include <pcl/io/pcd_io.h>

class VirtualSensor : public Sensor {
public:

    explicit VirtualSensor(const std::string& filenamePcd, const std::string& filenameFeaturePoints) : Sensor() {
        // load point cloud from file
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filenamePcd, *m_cloud) == -1) {
            std::cerr << "Couldn't read the pcd file " << filenamePcd << std::endl;
            exit(-1);
        }

		// load feature points from file
		FeaturePointExtractor inputFeatureExtractor(filenameFeaturePoints, m_cloud);
		m_featurePoints = inputFeatureExtractor.m_points;
    };

};