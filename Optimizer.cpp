#include "stdafx.h"
#include "Optimizer.h"

using namespace Eigen;

// Constant to allow better compile-time optimization.
const unsigned int NUM_EIGEN_VEC = 160;

struct ResidualFunctor {
	// x is the source (pos mesh), y is the target (input cloud)
	ResidualFunctor(const pcl::PointXYZRGB& inputPoint, const PixelData& rasterizerResult, const VectorXf& averageFaceVertices, const MatrixXf& shapeBasis, const Matrix4f& pose)
		: inputPoint(inputPoint), rasterizerResult(rasterizerResult), averageFaceVertices(averageFaceVertices), shapeBasis(shapeBasis), pose(pose) {}

	template <typename T>
	bool operator()(T const* alpha, T* residual) const {
		if (!rasterizerResult.isValid) {
			// Skip pixels where Steve isn't rendered into.
			residual[0] = T(0);
			residual[1] = T(0);
			residual[2] = T(0);
			return true;
		}

		//std::cout << "    Evaluating residual " << &inputPoint << " at " << alpha[0] << " ...";

		T worldSpacePixel[] = { T(0), T(0), T(0) };

		// For each vertex that is part of the triangle at this pixel.
		for (int i = 0; i < 3; i++) {
			int vertexIndex = rasterizerResult.vertexIndices[i];
			double barycentricFactor = rasterizerResult.barycentricCoordinates[i];

			// Vertex position of average face.
			T pos[] = {
				T(averageFaceVertices(3 * vertexIndex + 0)),
				T(averageFaceVertices(3 * vertexIndex + 1)),
				T(averageFaceVertices(3 * vertexIndex + 2)),
			};

			// Displace by applying alpha.
			for (int j = 0; j < shapeBasis.cols(); j++) {
				pos[0] += T(shapeBasis(3 * vertexIndex + 0, j)) * alpha[j];
				pos[1] += T(shapeBasis(3 * vertexIndex + 1, j)) * alpha[j];
				pos[2] += T(shapeBasis(3 * vertexIndex + 2, j)) * alpha[j];
			}

			// Transform to world space.
			T worldPos[] = {
				T(pose(0,0)) * pos[0] + T(pose(0,1)) * pos[1] + T(pose(0,2)) * pos[2] + T(pose(0,3)),
				T(pose(1,0)) * pos[0] + T(pose(1,1)) * pos[1] + T(pose(1,2)) * pos[2] + T(pose(1,3)),
				T(pose(2,0)) * pos[0] + T(pose(2,1)) * pos[1] + T(pose(2,2)) * pos[2] + T(pose(2,3)),
			};

			worldSpacePixel[0] += barycentricFactor * worldPos[0];
			worldSpacePixel[1] += barycentricFactor * worldPos[1];
			worldSpacePixel[2] += barycentricFactor * worldPos[2];
		}

		residual[0] = T(inputPoint.x) - worldSpacePixel[0];
		residual[1] = T(inputPoint.y) - worldSpacePixel[1];
		residual[2] = T(inputPoint.z) - worldSpacePixel[2];

		std::cout << " Done!" << std::endl;

		// TODO: point to plane distance, but for this we need normals
		// TODO: compare colors once we process albedo
		return true;
	}

private:
	// Input pixel that this residual is computing.
	const pcl::PointXYZRGB& inputPoint;

	// Shape (3 * numVertices,)
	const VectorXf& averageFaceVertices;
	// Shape (3 * numVertices, numEigenVec)
	const MatrixXf& shapeBasis;
	const Matrix4f& pose;

	// Rasterization result for this pixel.
	const PixelData& rasterizerResult;
};

struct RasterizerFunctor : public ceres::IterationCallback {
	RasterizerFunctor(std::vector<PixelData>& rasterizerResults, const double* alpha, const FaceModel& model, const Matrix4f& pose, const Matrix3f& intrinsics)
		: rasterizerResults(rasterizerResults), alpha(alpha), model(model), pose(pose), intrinsics(intrinsics) {}

	virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
		const Matrix3Xf& projectedVertices = project();
		rasterize(projectedVertices);
		return ceres::CallbackReturnType::SOLVER_CONTINUE;
	}

private:
	Matrix3Xf project() {
		std::cout << "Rasterization: project ..." << std::flush;

		FaceParameters params;
		params.alpha = Map<const VectorXd>(alpha, NUM_EIGEN_VEC).cast<float>();

		VectorXf flatVertices = model.computeShape(params);
		Matrix3Xf worldVertices = pose.topLeftCorner<3, 3>() * Map<Matrix3Xf>(flatVertices.data(), 3, model.getNumVertices());
		worldVertices.colwise() += pose.topRightCorner<3, 1>();

		// Project to pixel space.
		return intrinsics * worldVertices;
	}

	void rasterize(const Matrix3Xf& projectedVertices) {
		// Reset output.
		std::fill(rasterizerResults.begin(), rasterizerResults.end(), PixelData());

		std::cout << " rasterize ..." << std::flush;
		const Matrix3Xi& triangles = model.m_averageShapeMesh.triangles;
		for (size_t t = 0; t < triangles.cols(); t++) {
			Vector3f v0 = projectedVertices.col(triangles(0, t));
			Vector3f v1 = projectedVertices.col(triangles(1, t));
			Vector3f v2 = projectedVertices.col(triangles(2, t));

			if (t == 0) {
				std::cout << "first triangle: " << std::endl << v0.transpose() << std::endl << v1.transpose() << std::endl << v2.transpose() << std::endl;
			}

			// TODO actually do rasterization
		}

		// Add some dummy results for now.
		rasterizerResults[1 * 960 + 193] = { {1, 3, 5}, {0.1, 0.3, 0.6}, true };
		rasterizerResults[1 * 960 + 225] = { {25, 3246, 1231}, {0.2, 0.4, 0.4}, true };
		rasterizerResults[1 * 960 + 274] = { {62, 22, 44}, {0.45, 0.15, 0.4}, true };

		std::cout << "done!" << std::endl;
	}

	const double* alpha;
	const FaceModel& model;
	const Matrix4f& pose;
	const Matrix3f& intrinsics;

	// Output of the rasterization for all pixels (barycentric coordinates and vertex indices).
	// Stores info about pixels in row-major order, so pixel (x,y) is at index (y*width + x).
	std::vector<PixelData>& rasterizerResults;
};

FaceParameters optimizeParameters(FaceModel& model, const Matrix4f& pose, const Sensor& inputSensor) {
	const uint32_t width = inputSensor.m_cloud->width;
	const uint32_t height = inputSensor.m_cloud->height;
	std::array<double, NUM_EIGEN_VEC> alpha{};
	std::vector<PixelData> rasterResults(width * height);

	// Set up the rasterizer, which will be called once for each Ceres iteration and 
	// which updates rasterResults with the current per-pixel rendering results.
	RasterizerFunctor rasterizerCallback(rasterResults, alpha.data(), model, pose, inputSensor.m_cameraIntrinsics);
	// Initially call rasterizer once as the callback is only invoked AFTER each iteration.
	rasterizerCallback(ceres::IterationSummary());

	ceres::Problem problem;
	for (unsigned int y = 0; y < height; y++) {
		for (unsigned int x = 0; x < width; x++) {
			const pcl::PointXYZRGB& point = (*inputSensor.m_cloud)(x, y);
			if (std::isnan(point.z)) {
				continue;
			}

			// Temporary until we no longer use dummy data for the rasterizer.
			if (!rasterResults[y*width + x].isValid) continue;

			ceres::CostFunction* costFunc = new ceres::AutoDiffCostFunction<ResidualFunctor, 3, NUM_EIGEN_VEC>(
				new ResidualFunctor(point, rasterResults[y * width + x], model.m_averageShapeMesh.vertices, model.m_shapeBasis, pose));
			problem.AddResidualBlock(costFunc, NULL, alpha.data());
		}
	}
	std::cout << "Cost function has " << problem.NumResidualBlocks() << " residual blocks." << std::endl;

	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.update_state_every_iteration = true;
	options.callbacks.push_back(&rasterizerCallback);
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.FullReport() << std::endl;
	std::cout << "Some final values of alpha: " << Map<VectorXd>(alpha.data(), 10) << std::endl;

	FaceParameters params;
	params.alpha = Map<VectorXd>(alpha.data(), NUM_EIGEN_VEC).cast<float>();

	// For now, don't actually return the values because they don't make sense yet.
	params.alpha.setZero();
	return params;
}
