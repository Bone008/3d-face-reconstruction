#pragma once
#include "FaceModel.h"
#include "Sensor.h"

// Output of the rasterizer for a single pixel.
// Does not include the actual color & depth output (what a typical rasterizer would compute),
// because those are a function of the 3 vertices and thus need to be differentiable and
// computed individually by the residual functors.
struct PixelData {
	int vertexIndices[3];
	double barycentricCoordinates[3];
	bool isValid;
};

FaceParameters optimizeParameters(FaceModel& model, const Eigen::Matrix4f& pose, const Sensor& inputSensor);

// Rasterizes the entire synthetic model and computes barycentric coordinates and vertex indices for each pixel.
// Returns info about pixels in row-major order, so pixel (x,y) is at index (y*width + x).
std::vector<PixelData> rasterizeVertices(const double* vertices, const Eigen::MatrixX3i& triangleList, int width, int height);
