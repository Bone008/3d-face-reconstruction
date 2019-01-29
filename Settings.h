#pragma once

// Stores command line parameters.
struct Settings {
	std::string inputFile;
	
	bool skipOptimization;
	
	unsigned int optimizationStride;
	float regStrengthAlpha;
	float regStrengthBeta;
	float regStrengthGamma;
	double initialStepSize;
	double maxStepSize;
};

extern Settings gSettings;
