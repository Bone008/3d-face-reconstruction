#include <Eigen/Eigen>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "FeaturePointExtractor.h"
#include "Sensor.h"
#include "VirtualSensor.h"

int main(int argc, char **argv) {
    // filenames
    std::string filenameBase = "../data/rgbd_face_dataset/"; // TODO maybe change
    std::string filenamePcd = filenameBase + "006_00_cloud.pcd";
    std::string filenameIndices = filenameBase + "006_00_features.points";

    // load point cloud
    Sensor sensor = VirtualSensor(filenamePcd);

    // load feature points
    FeaturePointExtractor extractor(filenameIndices, sensor);

    // TODO continue
}