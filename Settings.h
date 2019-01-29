#pragma once

// Stores command line parameters.
struct Settings {
	std::string inputFile;
	std::string kinectOutputFile;
	bool useKinect;
	bool skipOptimization;
	
	unsigned int optimizationStride;
	float regStrengthAlpha;
	float regStrengthBeta;
	double initialStepSize;
	double maxStepSize;
};

extern Settings gSettings;
