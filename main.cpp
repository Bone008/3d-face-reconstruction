#include "stdafx.h"
#include "VirtualSensor.h"
#include "FaceModel.h"
#include "CoarseAlignment.h"
#include "Optimizer.h"
#include "utils.h"

const std::string baseModelDir = "../data/MorphableModel/";
const std::string inputFaceBaseDir = "../data/rgbd_face_dataset/";
const std::string inputFacePcdFile = inputFaceBaseDir + "006_00_cloud.pcd";
const std::string inputFeaturePointsFile = inputFaceBaseDir + "006_00_features.points";

pcl::visualization::PCLVisualizer viewer("PCL Viewer");

void highlightFeaturePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<Eigen::Vector3f> &featurePoints,
                            const std::string &name) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_highlight(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (auto const &point: featurePoints) {
        pcl::PointXYZRGB selected_point;
        selected_point.x = point[0];
        selected_point.y = point[1];
        selected_point.z = point[2];
        selected_point.r = 255;
        selected_point.g = 0;
        selected_point.b = 0;
        points_to_highlight->points.push_back(selected_point);
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> red(points_to_highlight);
    viewer.addPointCloud<pcl::PointXYZRGB>(points_to_highlight, red, name);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, name);
}


int main(int argc, char **argv) {
	std::cout << "Loading face model ..." << std::endl;
	FaceModel model(baseModelDir);
	std::cout << "Loading input data ..." << std::endl;
	Sensor inputSensor = VirtualSensor(inputFacePcdFile, inputFeaturePointsFile);
	
	// visualize input point cloud (John)
	viewer.addPointCloud<pcl::PointXYZRGB>(inputSensor.m_cloud, "inputCloud");
	highlightFeaturePoints(inputSensor.m_cloud, inputSensor.m_featurePoints, "inputCloudFeatures");

	std::cout << "Coarse alignment ..." << std::endl;
	Eigen::Matrix4f pose = computeCoarseAlignment(model, inputSensor);
	std::cout << "Optimizing parameters ..." << std::endl;
	FaceParameters params = optimizeParameters(model, pose, inputSensor);
	Eigen::VectorXf finalShape = model.computeShape(params);

	std::cout << "Done!" << std::endl;

	// visualize final reconstruction (Steve)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::transformPointCloud(*pointsToCloud(finalShape), *transformedCloud, pose);
	viewer.addPointCloud(transformedCloud, "finalCloud");

	viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);
	while (!viewer.wasStopped()) {
		// TODO: react to input to modify params and call viewer.updatePointCloud(...)
		viewer.spinOnce(500);
	}
	return 0;
}
