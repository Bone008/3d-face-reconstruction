#pragma once
#include "FaceModel.h"
#include "Sensor.h"

// Output of the rasterizer for a single pixel.
// Does not include the actual color & depth output (what a typical rasterizer would compute),
// because those are a function of the 3 vertices and thus need to be differentiable and
// computed individually by the residual functors.
struct PixelData {
	int vertexIndices[3];
	Eigen::Vector3f barycentricCoordinates;
	bool isValid;
};

FaceParameters optimizeParameters(FaceModel& model, const Eigen::Matrix4f& pose, const Sensor& inputSensor);
