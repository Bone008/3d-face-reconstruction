#include "OpenNI2Sensor.h"
#include "FeaturePointExtractor.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>

OpenNI2Sensor::OpenNI2Sensor(const std::string& filenamePcd, const std::string& filenameFeaturePoints) : Sensor() {
    if (!boost::filesystem::exists(filenamePcd)) {
        // remove previous feature points file as it will be invalid with new point cloud
        if (boost::filesystem::exists(filenameFeaturePoints))
            boost::filesystem::remove(filenameFeaturePoints);

        capturePointCloudToFile(filenamePcd);
    }

    loadFromFile(filenamePcd);

    // load feature points from file
    FeaturePointExtractor inputFeatureExtractor(filenameFeaturePoints, m_cloud);
    m_featurePoints = inputFeatureExtractor.m_points;

    // taken from https://codeyarns.com/2015/09/08/how-to-compute-intrinsic-camera-matrix-for-a-camera/
    // not sure if correct
    m_cameraIntrinsics << 583.2829786373293, 0.0, 320.0,
    0.0, 579.4112549695428, 240.0,
    0.0, 0.0, 1.0;
}

void OpenNI2Sensor::capturePointCloudToFile(const std::string& filenamePcd) {
    // FIXME passing the point cloud directly didn't work, so it is saved to a file and loaded from there
    std::cout << "  from sensor to file ..." << std::endl;

    pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
    pcl::visualization::CloudViewer viewer("Pick point cloud");
    PointCloudT outputCloud;

    // callback when cloud is available
    interface->registerCallback<void(const PointCloudT&)>([&](const PointCloudT& cloud) {
        if (!viewer.wasStopped()) {
            viewer.showCloud(cloud);
            outputCloud = cloud;
        }
    });

    // start capturing point clouds
    interface->start ();
    while (!viewer.wasStopped()){
        boost::this_thread::sleep(boost::posix_time::millisec(500));
    }
    interface->stop ();

    // save to file for later loading (TODO directly pass point cloud)
    std::cout << "Saving captured point cloud to file ..." << std::endl;
    pcl::io::savePCDFileBinary(filenamePcd, *outputCloud);
}

void OpenNI2Sensor::loadFromFile(const std::string& filenamePcd) {
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filenamePcd, *m_cloud) == -1) {
        std::cerr << "Couldn't read the pcd file " << filenamePcd << std::endl;
        exit(-1);
    }
}
