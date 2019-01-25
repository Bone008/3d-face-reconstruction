#pragma once

// Stores command line parameters.
struct Settings {
	std::string inputFile = "../data/rgbd_face_dataset/006_00_cloud.pcd";
	
	bool skipOptimization = false;
	
	unsigned int optimizationStride = 2;
	float regStrengthAlpha = 1.0f;
	float regStrengthBeta = 1.0f;
	double initialStepSize = 0.1;
	double maxStepSize = 0.25;
};

extern Settings gSettings;
