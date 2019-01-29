#include "stdafx.h"
#include "Settings.h"
#include "VirtualSensor.h"
#include "FaceModel.h"
#include "CoarseAlignment.h"
#include "Optimizer.h"
#include "utils.h"
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
			("input", "Input point cloud file (*.pcl).", cxxopts::value(gSettings.inputFile)->default_value("../data/rgbd_face_dataset/006_00_cloud.pcd"))
			("p,point-size", "Display size of the points of the input point cloud.", cxxopts::value(gSettings.inputCloudPointSize)->default_value("2.0"))
			("o,skip-optimization", "Skip fine optimization of face parameters completely.", cxxopts::value(gSettings.skipOptimization)->default_value("false"))
			("opt-stride", "Pixel stride for fine optimization (>= 1).", cxxopts::value(gSettings.optimizationStride)->default_value("2"))
			("s,opt-step", "Initial trust region size of the optimization.", cxxopts::value(gSettings.initialStepSize)->default_value("0.1"))
			("S,opt-max-step", "Maximum trust region size of the optimization.", cxxopts::value(gSettings.maxStepSize)->default_value("0.25"))
			("r,opt-reg-alpha", "Regularization strength for alpha parameters.", cxxopts::value(gSettings.regStrengthAlpha)->default_value("1.0"))
			("R,opt-reg-beta", "Regularization strength for beta parameters.", cxxopts::value(gSettings.regStrengthBeta)->default_value("1.0"))
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
    FaceParameters paramsDefault;
    FaceParameters params;
    Eigen::Matrix4f poseWithoutICP;
    Eigen::Matrix4f pose;

    FaceParameters currentParams = params;
    Eigen::Matrix4f currentPose = pose;
    bool optimized = true;
    int currentAge = 0;
    int currentWeight = 0;
    int currentGender = 0;
    bool albedoOff = false;


    auto renderPcl = [&]() {
        if (!optimized)
            currentParams = model->createDefaultParameters();
        if (albedoOff)
            currentParams.beta.setZero();

        FaceParameters attributeParams = model->computeShapeAttribute(currentParams, currentAge, currentWeight, currentGender);

        Eigen::VectorXf finalShape = model->computeShape(attributeParams);
        Eigen::Matrix4Xi finalColors = model->computeColors(attributeParams);

        // visualize final reconstruction (Steve)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointsToCloud(finalShape, finalColors), *transformedCloud, currentPose);
        visualizer.setStevePcl(transformedCloud);
    };


    auto setAndRenderPcl = [&](FaceParameters params) {
        currentParams = params;
        renderPcl();
    };

    boost::thread optimizingThread([&]() {
        // load face model (Steve)
        std::cout << "Loading face model ..." << std::endl;
        model = new FaceModel(baseModelDir);
        paramsDefault = model->createDefaultParameters();
        visualizer.setSteveVertices(trianglesToVertexList(model->m_averageMesh.triangles));

        std::cout << "Coarse alignment ..." << std::endl;
        poseWithoutICP = computeCoarseAlignmentProcrustes(*model, inputSensor);
        pose = computeCoarseAlignmentICP(*model, inputSensor, poseWithoutICP);
        currentPose = pose;
        visualizer.setCameraPose(&pose);

        if (gSettings.skipOptimization) {
            std::cout << "Skipping parameter optimization." << std::endl;
            params = model->createDefaultParameters();
        }
        else {
            std::cout << "Optimizing parameters ..." << std::endl;
            params = optimizeParameters(*model, pose, inputSensor, setAndRenderPcl);
        }

        std::cout << "Done!" << std::endl;
        setAndRenderPcl(params);
    });

    // add switch (optimized/default)
    std::vector<std::string> states;
	states.emplace_back("Optimized");
	states.emplace_back("Default");
    SwitchControl* scOptim = new SwitchControl(states, "", "Tab", [&](int state, const std::vector<int>&props) {
        optimized = (state == 0);
        renderPcl();
    });
	visualizer.addSwitch(scOptim);

	// add switch (icp on/off)
    std::vector<std::string> statesICP;
    statesICP.emplace_back("with ICP");
    statesICP.emplace_back("without ICP");
    SwitchControl* scICP = new SwitchControl(statesICP, "", "i", [&](int state, const std::vector<int>&props) {
        currentPose = (state == 0 ? pose : poseWithoutICP);
        renderPcl();
    });
    visualizer.addSwitch(scICP);

    // add switch (albedo on/off)
    std::vector<std::string> statesAlbedo;
    statesAlbedo.emplace_back("with albedo");
    statesAlbedo.emplace_back("without albedo");
    SwitchControl* scAlbedo = new SwitchControl(statesAlbedo, "", "space", [&](int state, const std::vector<int>&props) {
        albedoOff = (state != 0);
        renderPcl();
    });
    visualizer.addSwitch(scAlbedo);

    // add switch (age/gender/weight)
    std::vector<std::string> statesAttribute;
    SwitchControl* scAttribute = new SwitchControl(statesAttribute, "Left", "Right", [&](int state, const std::vector<int>&props) {
        currentAge = props[0];
        currentWeight = props[1];
        currentGender = props[2];
        renderPcl();
    });
    visualizer.addSwitch(scAttribute);

    visualizer.run();
    optimizingThread.join();

	return 0;
}
