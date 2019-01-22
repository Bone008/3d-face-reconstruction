#include "stdafx.h"
#include "Settings.h"
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

const std::string baseModelDir = "../data/MorphableModel/";

Settings gSettings;
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

void highlightFeaturePoints(std::vector<Eigen::Vector3f> &featurePoints, const std::string &name) {
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
	try {
		cxxopts::Options options(argv[0], "Program to reconstruct faces from RGB-D images.");
		options.add_options()
			("help", "Print help.")
			("input", "Input point cloud file (*.pcl).", cxxopts::value(gSettings.inputFile))
			("o,skip-optimization", "Enable fine optimization of face parameters.", cxxopts::value(gSettings.skipOptimization))
			("opt-stride", "Pixel stride for fine optimization.", cxxopts::value(gSettings.optimizationStride))
			("r,reg-alpha", "Regularization strength for alpha parameters.", cxxopts::value(gSettings.regStrengthAlpha))
			("initial-step-size", "Maximum trust region size of the optimization.", cxxopts::value(gSettings.initialStepSize))
			("s,max-step-size", "Maximum trust region size of the optimization.", cxxopts::value(gSettings.maxStepSize))
			;
		options.parse_positional("input");
		options.positional_help("[input]").show_positional_help();

		auto result = options.parse(argc, argv);
		if (result.count("help")) {
			std::cout << options.help() << std::endl;
			return 0;
		}
	}
	catch (cxxopts::OptionException e) {
		std::cerr << e.what() << std::endl;
		return -2;
	}

	std::string inputFace = gSettings.inputFile;
	std::string inputFeatures = inputFace.substr(0, inputFace.length() - 3) + "points";
	std::cout << "Loading input data ..." << std::endl;
	std::cout << "    Input file: " << inputFace << std::endl;
	Sensor inputSensor = VirtualSensor(inputFace, inputFeatures);
	
	// visualize input point cloud (John)
	viewer.addPointCloud<pcl::PointXYZRGB>(inputSensor.m_cloud, "inputCloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "inputCloud");
	highlightFeaturePoints(inputSensor.m_featurePoints, "inputCloudFeatures");


	std::cout << "Loading face model ..." << std::endl;
	FaceModel model(baseModelDir);

	std::cout << "Coarse alignment ..." << std::endl;
	Eigen::Matrix4f pose = computeCoarseAlignment(model, inputSensor);
	
	FaceParameters params;
	if (gSettings.skipOptimization) {
		std::cout << "Skipping parameter optimization." << std::endl;
		params = model.createDefaultParameters();
	}
	else {
		std::cout << "Optimizing parameters ..." << std::endl;
		params = optimizeParameters(model, pose, inputSensor);
	}

	Eigen::VectorXf finalShape = model.computeShape(params);
	Eigen::Matrix4Xi finalColors = model.computeColors(params);

	std::cout << "Done!" << std::endl;

	// visualize final reconstruction (Steve)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::transformPointCloud(*pointsToCloud(finalShape, finalColors), *transformedCloud, pose);
	viewer.addPolygonMesh<pcl::PointXYZRGB>(transformedCloud, trianglesToVertexList(model.m_averageMesh.triangles), "steveMesh");

	std::vector<std::string> states;
	states.emplace_back("Optimized");
	states.emplace_back("Default");

	FaceParameters defaultParams = model.createDefaultParameters();

	SwitchControl sc(viewer, states, "", "Tab", [&](int state, const std::vector<int>&props) {
		std::cout << "Switching to " << (state == 0 ? "optimized" : "default") << " face." << std::endl;

		FaceParameters newParams = (state == 0 ? params : defaultParams);
		FaceParameters outparams = model.computeShapeAttribute(newParams, props[0], props[1], props[2]);

		Eigen::VectorXf finalShape = model.computeShape(outparams);
		Eigen::Matrix4Xi finalColors = model.computeColors(outparams);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*pointsToCloud(finalShape, finalColors), *transformedCloud, pose);
		viewer.updatePolygonMesh<pcl::PointXYZRGB>(transformedCloud, trianglesToVertexList(model.m_averageMesh.triangles), "steveMesh");
	});

	// Make camera look at the target.
	Eigen::Vector4f objectOrigin = pose * Eigen::Vector4f(0, 0, 0, 1);
	Eigen::Vector4f cameraPos = objectOrigin + Eigen::Vector4f(0, 0, -0.7f, 0);
	viewer.setCameraPosition(cameraPos.x(), cameraPos.y(), cameraPos.z(), objectOrigin.x(), objectOrigin.y(), objectOrigin.z(), 0, -1, 0);

	while (!viewer.wasStopped()) {
		viewer.spinOnce(500);
	}
	return 0;
}
