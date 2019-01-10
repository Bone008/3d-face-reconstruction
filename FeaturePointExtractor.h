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

    FeaturePointExtractor(const std::string& filenameIndices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        // check if 5 feature points' indices exist
        if (!boost::filesystem::exists(filenameIndices)) {
            // manual selection
            std::cout << "Pick feature points using shift+click!" << std::endl;
            manualFeaturePointSelection(cloud, filenameIndices);
        }

        loadFromFile(filenameIndices);
    }

private:

    void manualFeaturePointSelection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filenameIndices) {
        std::ofstream outfile;
        outfile.open(filenameIndices);

        pcl::visualization::PCLVisualizer viewer("Pick feature points");

        // Draw output point cloud:
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");

        viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);
        int count = 0;
        bool stopped = false;
        viewer.registerPointPickingCallback([&](const pcl::visualization::PointPickingEvent& event) {
            int pInd = event.getPointIndex();
            if (pInd == -1)
                return;

            float x, y, z;
            event.getPoint(x, y, z);
            std::cout << "  Picked point: " << x << " " << y << " " << z << std::endl;
            outfile << x << " " << y << " " << z << std::endl;

            count++;
            if (count >= 5) {
                stopped = true;
            }
        });

        while (!viewer.wasStopped() && !stopped) {
            viewer.spinOnce(500);
        }
        viewer.close();

        outfile.close();
    }

    void loadFromFile(const std::string& filenameIndices) {
        std::ifstream fileIndices(filenameIndices, std::ios::in);
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