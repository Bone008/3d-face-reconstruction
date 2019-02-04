#include "stdafx.h"
#include "Settings.h"
#include "VirtualSensor.h"
#include "OpenNI2Sensor.h"
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
            ("k,use-kinect", "Acquire point cloud from kinect sensor.", cxxopts::value(gSettings.useKinect)->default_value("false"))
            ("kinect-output", "Output point cloud file (*.pcl) when using kinect sensor.", cxxopts::value(gSettings.kinectOutputFile)->default_value("../data/sensor_output.pcd"))
			("p,point-size", "Display size of the points of the input point cloud.", cxxopts::value(gSettings.inputCloudPointSize)->default_value("2.0"))
			("o,skip-optimization", "Skip fine optimization of face parameters completely.", cxxopts::value(gSettings.skipOptimization)->default_value("false"))
			("n,num-linear-iterations", "Number of initialization iterations of linear shape solver (>= 0).", cxxopts::value(gSettings.numLinearIterations)->default_value("5"))
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

	std::cout << "Loading input data ..." << std::endl;
	Sensor* inputSensorPtr;
    if (gSettings.useKinect) {
        std::string outputFace = gSettings.kinectOutputFile;
        std::string outputFeatures = outputFace.substr(0, outputFace.length() - 3) + "points";

        std::cout << "    From sensor" << std::endl;
        inputSensorPtr = new OpenNI2Sensor(outputFace, outputFeatures);
    } else {
        std::string inputFace = gSettings.inputFile;
        std::string inputFeatures = inputFace.substr(0, inputFace.length() - 3) + "points";

        std::cout << "    Input file: " << inputFace << std::endl;
        inputSensorPtr = new VirtualSensor(inputFace, inputFeatures);
    }
    Sensor& inputSensor = *inputSensorPtr;


	std::string inputPclFile = (gSettings.useKinect ? gSettings.kinectOutputFile : gSettings.inputFile);
	std::string inputRgbFile = inputPclFile.substr(0, inputPclFile.length() - 4) + "_color.bmp";
	std::string inputDepthFile = inputPclFile.substr(0, inputPclFile.length() - 4) + "_depth.bmp";
	std::cout << "Saving input RGB to " << inputRgbFile << " ..." << std::endl;
    saveBitmap(inputRgbFile.c_str(), inputSensor.m_cloud->width, inputSensor.m_cloud->height, [&](unsigned int x, unsigned int y) {
        y = inputSensor.m_cloud->height - y - 1;

        const pcl::PointXYZRGB& inputPixel = (*inputSensor.m_cloud)(x, y);
        return inputPixel.getRGBVector3i();
    });

	std::cout << "Saving input depth to " << inputDepthFile << " ..." << std::endl;
    float maxDepth = 0.5f;
    saveBitmap(inputDepthFile.c_str(), inputSensor.m_cloud->width, inputSensor.m_cloud->height, [&](unsigned int x, unsigned int y) {
        y = inputSensor.m_cloud->height - y - 1;

        const pcl::PointXYZRGB& inputPixel = (*inputSensor.m_cloud)(x, y);
        int depth = (int)((1.0f-inputPixel.z) / maxDepth * 255);
        if (depth < 0)
            depth = 0;
        else if (depth > 255)
            depth = 255;
        return Eigen::Vector3i(depth, depth, depth);
    });

	// visualize input point cloud (John)
	visualizer.setJohnPcl(inputSensor.m_cloud);
    visualizer.setJohnFeatures(getFeaturePointPcl(inputSensor.m_featurePoints));
    visualizer.runOnce();

    FaceModel* model = nullptr;
	OptimizerOutput* optimizerData = nullptr;
    
	// Updated by the optimizer thread during optimization.
	FaceParameters params;
    // Modified by renderPcl() depending on settings controlled by user input.
	FaceParameters currentParams;

    Eigen::Matrix4f poseWithoutICP;
    Eigen::Matrix4f pose;
    Eigen::Matrix4f currentPose = pose;

    bool shapeOff = false;
    bool albedoOff = false;
	bool whiteColor = false;
    int currentAge = 0;
    int currentWeight = 0;
    int currentGender = 0;


    auto renderPcl = [&]() {
		if (model == nullptr)
			return; // Model not loaded yet.
        
		currentParams = params;
		if (shapeOff)
			currentParams.alpha.setZero();
        if (albedoOff)
            currentParams.beta.setZero();
        currentParams = model->computeShapeAttribute(currentParams, currentAge, currentWeight, currentGender);

        Eigen::VectorXf finalShape = model->computeShape(currentParams);
        Eigen::Matrix4Xi finalColors = model->computeColors(currentParams);

		if (whiteColor)
			finalColors.setConstant(255);

        // visualize final reconstruction (Steve)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointsToCloud(finalShape, finalColors), *transformedCloud, currentPose);
        visualizer.setStevePcl(transformedCloud);
    };


	auto saveComposite = [&]() {
		if (optimizerData == nullptr) {
			// Optimizer hasn't completed yet, so no rasterized output is available.
			std::cout << "Cannot save composite image because optimizer has not finished." << std::endl;
			return;
		}

		std::string filename = (gSettings.useKinect ? gSettings.kinectOutputFile : gSettings.inputFile);
		filename = filename.substr(0, filename.length() - 4) + "_composite.bmp";
		std::cout << "Saving composite image to " << filename << " ..." << std::endl;
		saveCompositeImage(filename.c_str(), *inputSensor.m_cloud, *optimizerData, currentParams);
	};

    boost::thread optimizingThread([&]() {
		auto setAndRenderPcl = [&](const FaceParameters& newParams) {
			params = newParams;
			renderPcl();
		};

        // load face model (Steve)
        std::cout << "Loading face model ..." << std::endl;
        model = new FaceModel(baseModelDir);
		params = model->createDefaultParameters();
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
			OptimizerOutput* wipData = new OptimizerOutput;
            params = optimizeParameters(*model, pose, inputSensor, *wipData, setAndRenderPcl);
			
			// After optimization is completed, save state to shared pointer.
			optimizerData = wipData;
        }

        std::cout << "Done!" << std::endl;
        setAndRenderPcl(params);
		saveComposite();
    });

    // add switch (optimized/default)
    std::vector<std::string> states;
	states.emplace_back("Optimized");
	states.emplace_back("Optimized (only shape)");
	states.emplace_back("Optimized (white shape)");
	states.emplace_back("Optimized (only albedo)");
	states.emplace_back("Default");
    SwitchControl* scOptim = new SwitchControl(states, "", "Tab", [&](int state, const std::vector<int>&props) {
		switch (state) {
		case 0: shapeOff = false; albedoOff = false; whiteColor = false; break;
		case 1: shapeOff = false; albedoOff = true;  whiteColor = false; break;
		case 2: shapeOff = false; albedoOff = true;  whiteColor = true;  break;
		case 3: shapeOff = true;  albedoOff = false; whiteColor = false; break;
		case 4: shapeOff = true;  albedoOff = true;  whiteColor = false; break;
		}
        renderPcl();
    });
	visualizer.addSwitch(scOptim);

	// add switch (icp on/off)
    std::vector<std::string> statesICP;
    statesICP.emplace_back("Pose with ICP");
    statesICP.emplace_back("Pose without ICP");
    SwitchControl* scICP = new SwitchControl(statesICP, "", "i", [&](int state, const std::vector<int>&props) {
        currentPose = (state == 0 ? pose : poseWithoutICP);
        renderPcl();
    });
    visualizer.addSwitch(scICP);

    // add switch (age/gender/weight)
    std::vector<std::string> statesAttribute;
    SwitchControl* scAttribute = new SwitchControl(statesAttribute, "Left", "Right", [&](int state, const std::vector<int>&props) {
        currentAge = props[0];
        currentWeight = props[1];
        currentGender = props[2];
        renderPcl();
    });
    visualizer.addSwitch(scAttribute);
	
	std::vector<std::string> dummyStates{ "save composite" };
	SwitchControl* scSaveImage = new SwitchControl(dummyStates, "", "s", [&](auto dummy, auto dummy2) {
		saveComposite();
	});
	visualizer.addSwitch(scSaveImage);

    visualizer.run();
    optimizingThread.join();

	return 0;
}
