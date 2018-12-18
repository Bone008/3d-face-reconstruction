#include "stdafx.h"
#include "Optimizer.h"

FaceParameters optimize_parameters(FaceModel& model, const Eigen::Matrix4f& pose, const Sensor& inputSensor) {
	/* high-level overview of what should probably happen:

	alpha = 0
	for each pixel (i,j) in depth_map:
		init_residual(inputSensor.depth_map, i, j, model.shapeBasis, model.averageFace, pose, inputSensor.camera)

	for each ceres iteration:
		barycentric_buffer, vertex_index_buffer = rasterize_steve(inputSensor.intrinsics, model, pose)
		r.compute(barycentric_buffer, vertex_index_buffer)
	*/

	return FaceParameters();
}

std::vector<PixelData> rasterize_vertices(const double* vertices, const Eigen::MatrixX3i& triangleList, int width, int height)
{
	return std::vector<PixelData>();
}
