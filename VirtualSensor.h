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

		// constants from the test RGBD dataset
		m_cameraIntrinsics <<
			1052.667867276341, 0, 962.4130834944134,
			0, 1052.020917785721, 536.2206151001486,
			0, 0, 1;
		// since we care about the depth image and it is half the resolution of the color image,
		// we need to adjust the intrinsics accordingly
		m_cameraIntrinsics.topRows(2) /= 2;
    };

};