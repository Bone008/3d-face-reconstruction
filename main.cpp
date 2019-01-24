#include "stdafx.h"
#include "VirtualSensor.h"
#include "FaceModel.h"
#include "CoarseAlignment.h"
#include "Optimizer.h"
#include "utils.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include "SwitchControl.h"
#include "Visualizer.h"

const std::string baseModelDir = "../data/MorphableModel/";
const std::string inputFaceBaseDir = "../data/rgbd_face_dataset/";
const std::string inputFacePcdFile = inputFaceBaseDir + "006_00_cloud.pcd";
const std::string inputFeaturePointsFile = inputFaceBaseDir + "006_00_features.points";

// TODO add switch for feature points (on/off)
// TODO visualize intermediate output during optimization
// TODO add switch for different intermediate outputs
// TODO show progress on screen
Visualizer visualizer;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFeaturePointPcl(std::vector<Eigen::Vector3f> &featurePoints) {
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

    return points_to_highlight;
}


int main(int argc, char **argv) {
	// load input point cloud (John)
	std::cout << "Loading input data ..." << std::endl;
	Sensor inputSensor = VirtualSensor(inputFacePcdFile, inputFeaturePointsFile);
	visualizer.setJohnPcl(inputSensor.m_cloud);
	visualizer.setJohnFeatures(getFeaturePointPcl(inputSensor.m_featurePoints));
	visualizer.runOnce();

	FaceModel* model;
    FaceParameters params;
	Eigen::Matrix4f pose;
    boost::thread optimizingThread([&]() {
		// load face model (Steve)
		std::cout << "Loading face model ..." << std::endl;
		model = new FaceModel(baseModelDir);
		visualizer.setSteveVertices(trianglesToVertexList(model->m_averageShapeMesh.triangles));

        std::cout << "Coarse alignment ..." << std::endl;
        pose = computeCoarseAlignment(*model, inputSensor);

        std::cout << "Optimizing parameters ..." << std::endl;
        params = optimizeParameters(*model, pose, inputSensor);
		Eigen::VectorXf finalShape = model->computeShape(params);
        std::cout << "Done!" << std::endl;

        // visualize final reconstruction (Steve)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointsToCloud(finalShape, model->m_averageShapeMesh.vertexColors), *transformedCloud, pose);
        visualizer.setStevePcl(transformedCloud);
    });

	// add switch (optimized/default)
	std::vector<std::string> states;
	states.emplace_back("Optimized");
	states.emplace_back("Default");

	SwitchControl* sc = new SwitchControl(states, "", "Tab", [&](int state) {
		std::cout << "Switching to " << (state == 0 ? "optimized" : "default") << " face." << std::endl;

		FaceParameters newParams = params;
		if (state == 1 /* Default */) {
			newParams.alpha.setZero();
		}

		Eigen::VectorXf finalShape = model->computeShape(newParams);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*pointsToCloud(finalShape, model->m_averageShapeMesh.vertexColors), *transformedCloud, pose);
		visualizer.setStevePcl(transformedCloud);

	});
	visualizer.addSwitch(sc);

	visualizer.run();
	optimizingThread.join();

	return 0;
}
