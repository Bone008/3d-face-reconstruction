#pragma once

#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Sensor.h"

/*
 * Cloud rendering adapted from:
 * http://robotics.dei.unipd.it/reid/index.php/8-dataset/9-overview-face
 */
class FeaturePointExtractor {
public:

    std::vector<pcl::PointXYZ> m_points;

    FeaturePointExtractor(const std::string &filenameIndices, Sensor &sensor) {
        // check if 5 feature points' indices exist
        std::ifstream fileIndices(filenameIndices, std::ios::in);

        if (!fileIndices.is_open()) {
            // manual selection
            std::cout << "Couldn't open indices files. Please pick points using shift+klick and write into file "
                      << filenameIndices << std::endl;
            manualFeaturePointSelection(sensor);
            exit(0);
        }

        loadFromFile(fileIndices);

        // 3D Visualization
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");

        // Draw output point cloud:
        viewer.addCoordinateSystem(0.1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(sensor.m_cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(sensor.m_cloud, rgb, "cloud");

        // Draw feature points in red:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_highlight(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (auto const &point: m_points) {
            pcl::PointXYZRGB selected_point;
            selected_point.x = point.x;
            selected_point.y = point.y;
            selected_point.z = point.z;
            selected_point.r = 255;
            selected_point.g = 0;
            selected_point.b = 0;
            points_to_highlight->points.push_back(selected_point);
        }

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> red(points_to_highlight);
        viewer.addPointCloud<pcl::PointXYZRGB>(points_to_highlight, red, "point");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "point");
        viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);

        while (not viewer.wasStopped()) {
            viewer.spinOnce();
            cv::waitKey(1);
        }
    }

    static void pointPickingHandler(const pcl::visualization::PointPickingEvent &event, void *viewer_void) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

        int pInd = event.getPointIndex();
        if (pInd == -1)
            return;

        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << "Picked point (" << x << ", " << y << ", " << z << ")" << std::endl;
        // FIXME a segfault is produced after picking a point
    }

private:

    void manualFeaturePointSelection(Sensor &sensor) {
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");

        // Draw output point cloud:
        viewer.addCoordinateSystem(0.1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(sensor.m_cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(sensor.m_cloud, rgb, "cloud");

        viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);
        viewer.registerPointPickingCallback(FeaturePointExtractor::pointPickingHandler, (void *) &viewer);

        while (not viewer.wasStopped()) {
            viewer.spinOnce();
            cv::waitKey(1);
        }
    }

    void loadFromFile(std::ifstream &fileIndices) {
        pcl::PointXYZ p;
        while (fileIndices >> p.x >> p.y >> p.z) {
            std::cout << p.x << "," << p.y << "," << p.z << std::endl;
            pcl::PointXYZ copy = p;
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