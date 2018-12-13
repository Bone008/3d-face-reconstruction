#pragma once

#include <fstream>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Sensor.h"

/*
 * Cloud rendering adapted from:
 * http://robotics.dei.unipd.it/reid/index.php/8-dataset/9-overview-face
 */
class FeaturePointExtractor {
public:

    std::vector<Eigen::Vector3f> m_points;

    FeaturePointExtractor(const std::string &filenameIndices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        // check if 5 feature points' indices exist
        std::ifstream fileIndices(filenameIndices, std::ios::in);

        if (!fileIndices.is_open()) {
            // manual selection
            std::cout << "Couldn't open indices files. Please pick points using shift+klick and write into file "
                      << filenameIndices << std::endl;
            manualFeaturePointSelection(cloud);
            exit(0);
        }

        loadFromFile(fileIndices);
    }

    static void pointPickingHandler(const pcl::visualization::PointPickingEvent &event, void *) {
        int pInd = event.getPointIndex();
        if (pInd == -1)
            return;

        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << x << " " << y << " " << z << std::endl;
    }

private:

    void manualFeaturePointSelection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");

        // Draw output point cloud:
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");

        viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);
        viewer.registerPointPickingCallback(FeaturePointExtractor::pointPickingHandler);

        while (!viewer.wasStopped()) {
            viewer.spinOnce(500);
        }
    }

    void loadFromFile(std::ifstream &fileIndices) {
        Eigen::Vector3f v;
        while (fileIndices >> v[0] >> v[1] >> v[2]) {
            Eigen::Vector3f copy = v;
            m_points.push_back(copy);
        }
        fileIndices.close();

        // check if 5 points exist
        if (m_points.size() != 5) {
            std::cerr << "Number of feature points must equal 5!" << std::endl;
            //exit(-1);
        }
    }
};