#include "stdafx.h"
#include "Optimizer.h"

using namespace Eigen;

struct ResidualFunctor {
	// x is the source (average mesh), y is the target (input cloud)
	ResidualFunctor(int px, int py, const PixelData& rasterizerResult, const VectorXf&& averageFaceVertices, const MatrixXf& shapeBasis, const Matrix4f& pose)
		: px(px), py(py), rasterizerResult(rasterizerResult), averageFaceVertices(averageFaceVertices), shapeBasis(shapeBasis), pose(pose) {}

	template <typename T>
	bool operator()(T const* alpha, T* residual) const {
		rasterizerResult.vertexIndices;

		// TODO do the things

		//T baseTimesAlpha = T(0);
		//for (int e = 0; e < nEigenVec_; e++) {
		//	baseTimesAlpha += T(base_(i_, e)) * alpha[e];
		//}

		//residual[0] = T(y_) - (T(x_) + baseTimesAlpha);
		return true;
	}

private:
	// Pixel positions of this residual.
	const int px, py;

	const VectorXf& averageFaceVertices;
	const MatrixXf& shapeBasis;
	const Matrix4f& pose;

	// Rasterization result for this pixel.
	const PixelData& rasterizerResult;

	// TODO missing: camera intrinsics, pointer to rasterization result, pointer to input sensor depth map
};

FaceParameters optimizeParameters(FaceModel& model, const Eigen::Matrix4f& pose, const Sensor& inputSensor) {
	/* high-level overview of what should probably happen:


	alpha = 0
	for each pixel (i,j) in depth_map:
		init_residual(inputSensor.depth_map, i, j, model.shapeBasis, model.averageFace, pose, inputSensor.camera)

	for each ceres iteration:
		barycentric_buffer, vertex_index_buffer = rasterize_steve(inputSensor.intrinsics, model, pose)
		r.compute(barycentric_buffer, vertex_index_buffer)
	*/

	ceres::Problem problem;

	unsigned int numPixels = 5;

	double* alpha = new double[model.getNumEigenVec()];
	for (unsigned int i = 0; i < numPixels; i++) {
		//ceres::CostFunction *cost_function =
		//	new ceres::AutoDiffCostFunction<ResidualFunctor, 1, 2>(
		//		new ResidualFunctor(/* TODO fill this */));
		//problem.AddResidualBlock(cost_function, NULL, alpha);
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	FaceParameters params;
	// for now, return 0
	params.alpha.setZero(model.getNumEigenVec());
	return params;
}

std::vector<PixelData> rasterizeVertices(const double* vertices, const Eigen::MatrixX3i& triangleList, int width, int height)
{
	return std::vector<PixelData>();
}
