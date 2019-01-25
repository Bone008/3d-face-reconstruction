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
#include "Visualizer.h"

const std::string baseModelDir = "../data/MorphableModel/";

Settings gSettings;

// TODO add switch for feature points (on/off)
// TODO add switch for different intermediate outputs
// TODO show progress on screen
// TODO exit on escape key (also stop optimization)
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
        visualizer.setSteveVertices(trianglesToVertexList(model->m_averageMesh.triangles));

        std::cout << "Coarse alignment ..." << std::endl;
        pose = computeCoarseAlignment(*model, inputSensor);
        visualizer.setCameraPose(&pose);

        auto renderPcl = [&](FaceParameters params) {
            Eigen::VectorXf finalShape = model->computeShape(params);
            Eigen::Matrix4Xi finalColors = model->computeColors(params);

            // visualize final reconstruction (Steve)
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*pointsToCloud(finalShape, finalColors), *transformedCloud, pose);
            visualizer.setStevePcl(transformedCloud);
        };

        if (gSettings.skipOptimization) {
            std::cout << "Skipping parameter optimization." << std::endl;
            params = model->createDefaultParameters();
        }
        else {
            std::cout << "Optimizing parameters ..." << std::endl;
            params = optimizeParameters(*model, pose, inputSensor, renderPcl);
        }

        std::cout << "Done!" << std::endl;
        renderPcl(params);
    });

    // add switch (optimized/default)
    std::vector<std::string> states;
	states.emplace_back("Optimized");
	states.emplace_back("Default");

    SwitchControl* sc = new SwitchControl(states, "", "Tab", [&](int state, const std::vector<int>&props) {
        FaceParameters defaultParams = model->createDefaultParameters();
        FaceParameters newParams = (state == 0 ? params : defaultParams);
        FaceParameters outparams = model->computeShapeAttribute(newParams, props[0], props[1], props[2]);

        Eigen::VectorXf finalShape = model->computeShape(outparams);
        Eigen::Matrix4Xi finalColors = model->computeColors(outparams);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointsToCloud(finalShape, finalColors), *transformedCloud, pose);
        visualizer.setStevePcl(transformedCloud);
    });
	visualizer.addSwitch(sc);

    visualizer.run();
    optimizingThread.join();

	return 0;
}
