#include "OpenNI2Sensor.h"
#include "FeaturePointExtractor.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>

OpenNI2Sensor::OpenNI2Sensor() : m_viewer("test"), Sensor() {
    pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

    // callback if cloud is available
    boost::function<void (const PointCloudT&)> f =
            boost::bind (&OpenNI2Sensor::cloud_cb, this, _1);
    interface->registerCallback (f);

    // start capturing point clouds
    interface->start ();
    while (!m_viewer.wasStopped()){
        boost::this_thread::sleep(boost::posix_time::millisec(500));
    }
    interface->stop ();

    // copy rgba data to rgb output pointcloud
    pcl::PointXYZRGB t;
    pcl::PointCloud<pcl::PointXYZRGB> tmp(m_cloud_rgba->width, m_cloud_rgba->height, t);
    for (size_t i = 0; i < m_cloud->points.size(); i++) {
        tmp.points[i].x = m_cloud_rgba->points[i].x;
        tmp.points[i].y = m_cloud_rgba->points[i].y;
        tmp.points[i].z = m_cloud_rgba->points[i].z;
        tmp.points[i].r = m_cloud_rgba->points[i].r;
        tmp.points[i].g = m_cloud_rgba->points[i].g;
        tmp.points[i].b = m_cloud_rgba->points[i].b;
    }
    m_cloud = tmp.makeShared();

    // save to file for later reuse
    std::cout << "Saving captured point cloud to file ..." << std::endl;
    pcl::io::savePCDFileBinary("test_pcd.pcd", *m_cloud);
}

void OpenNI2Sensor::cloud_cb (const PointCloudT &cloud) {
    if (!m_viewer.wasStopped()) {
        m_viewer.showCloud(cloud);
        m_cloud_rgba = cloud;
    }
}
