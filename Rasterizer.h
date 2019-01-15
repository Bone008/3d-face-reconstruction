#pragma once
#include "FaceModel.h"
#include "BMP.h"

// Output of the rasterizer for a single pixel.
// Does not include the actual color & depth output (what a typical rasterizer would compute),
// because those are a function of the 3 vertices and thus need to be differentiable and
// computed individually by the residual functors.
struct PixelData {
	int vertexIndices[3];
	// Center of this pixel in screen space (x+0.5, y+0.5).
	Eigen::Vector2f pixelCenter;
	Eigen::Vector3f barycentricCoordinates;
	Eigen::Vector3f albedo;
	bool isValid;
};

class Rasterizer {
public:
	const FaceModel& model;
	std::vector<PixelData> pixelResults;

	Rasterizer(Eigen::Array2i frameSize, const FaceModel& model, const Eigen::Matrix4f& pose, const Eigen::Matrix3f& intrinsics)
		: frameSize(frameSize), model(model), pose(pose), intrinsics(intrinsics),
		pixelResults(frameSize.x() * frameSize.y()), depthBuffer(frameSize.x(), frameSize.y()) {}

	void compute(const FaceParameters& params);
	Eigen::Vector3f getAverageColor();

private:
	const Eigen::Array2i frameSize;
	const Eigen::Matrix4f& pose;
	const Eigen::Matrix3f& intrinsics;

	int numCalls = 0;
	Eigen::ArrayXXf depthBuffer;

	void project(const FaceParameters& params, Eigen::Matrix3Xf& outProjectedVertices, Eigen::Matrix4Xi& outVertexAlbedos);
	void rasterize(const Eigen::Matrix3Xf& projectedVertices, const Eigen::Matrix4Xi& vertexAlbedos);

	void writeDebugImages();
};
