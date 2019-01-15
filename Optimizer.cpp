#include "stdafx.h"
#include <pcl/filters/crop_box.h>
#include "Optimizer.h"
#include "BMP.h"
#include "utils.h"

using namespace Eigen;

// Constant to allow better compile-time optimization.
// If this is smaller than the number of actual eigen vectors (160),
// only the first ones will be optimized over.
const unsigned int NUM_ALPHA_VEC = 80;
const unsigned int NUM_BETA_VEC = 20;

const unsigned int NUM_DENSE_RESIDUALS = 3 + 3;

struct ResidualFunctor {
	// x is the source (pos mesh), y is the target (input cloud)
	ResidualFunctor(const pcl::PointXYZRGB& inputPoint, const PixelData& rasterizerResult, const FaceModel& model, const Matrix4f& pose)
		: inputPoint(inputPoint), rasterizerResult(rasterizerResult), model(model), pose(pose) {}

	template <typename T>
	bool operator()(T const* alpha, T const* beta, T* residual) const {
		if (!rasterizerResult.isValid) {
			// Skip pixels where Steve isn't rendered into.
			std::fill(residual, residual + NUM_DENSE_RESIDUALS, T(0));
			return true;
		}

		T worldPos[] = { T(0), T(0), T(0) };
		T albedo[] = { T(0), T(0), T(0) };

		// For each vertex that is part of the triangle at this pixel.
		for (int i = 0; i < 3; i++) {
			int vertexIndex = rasterizerResult.vertexIndices[i];
			float barycentricFactor = rasterizerResult.barycentricCoordinates[i];

			// Vertex position of average face.
			T pos[] = {
				T(model.m_averageMesh.vertices(3 * vertexIndex + 0)),
				T(model.m_averageMesh.vertices(3 * vertexIndex + 1)),
				T(model.m_averageMesh.vertices(3 * vertexIndex + 2)),
			};

			// Displace by applying alpha.
			for (int j = 0; j < NUM_ALPHA_VEC; j++) {
				T std = T(model.m_shapeStd(j));
				pos[0] += T(model.m_shapeBasis(3 * vertexIndex + 0, j)) * std * alpha[j];
				pos[1] += T(model.m_shapeBasis(3 * vertexIndex + 1, j)) * std * alpha[j];
				pos[2] += T(model.m_shapeBasis(3 * vertexIndex + 2, j)) * std * alpha[j];
			}

			// Transform to world space.
			T vertexWorldPos[] = {
				T(pose(0,0)) * pos[0] + T(pose(0,1)) * pos[1] + T(pose(0,2)) * pos[2] + T(pose(0,3)),
				T(pose(1,0)) * pos[0] + T(pose(1,1)) * pos[1] + T(pose(1,2)) * pos[2] + T(pose(1,3)),
				T(pose(2,0)) * pos[0] + T(pose(2,1)) * pos[1] + T(pose(2,2)) * pos[2] + T(pose(2,3)),
			};

			// Albedo of average face (ignore alpha).
			T vertexAlbedo[] = {
				T(model.m_averageMesh.vertexColors(0, vertexIndex)),
				T(model.m_averageMesh.vertexColors(1, vertexIndex)),
				T(model.m_averageMesh.vertexColors(2, vertexIndex)),
			};

			// Apply beta to albedo.
			for (int j = 0; j < NUM_BETA_VEC; j++) {
				T std = T(model.m_albedoStd(j));
				vertexAlbedo[0] += T(model.m_albedoBasis(3 * vertexIndex + 0, j)) * std * beta[j];
				vertexAlbedo[1] += T(model.m_albedoBasis(3 * vertexIndex + 1, j)) * std * beta[j];
				vertexAlbedo[2] += T(model.m_albedoBasis(3 * vertexIndex + 2, j)) * std * beta[j];
			}

			// Accumulate using barycentric coords.
			T b = T(barycentricFactor);
			worldPos[0] += b * vertexWorldPos[0];
			worldPos[1] += b * vertexWorldPos[1];
			worldPos[2] += b * vertexWorldPos[2];
			albedo[0] += b * vertexAlbedo[0];
			albedo[1] += b * vertexAlbedo[1];
			albedo[2] += b * vertexAlbedo[2];
		}

		residual[0] = T(inputPoint.x) - worldPos[0];
		residual[1] = T(inputPoint.y) - worldPos[1];
		residual[2] = T(inputPoint.z) - worldPos[2];
		// TODO: point to plane distance, but for this we need normals

		T colorScaling = T(1.f / 255.f);
		residual[3] = colorScaling * (T(inputPoint.r) - albedo[0]);
		residual[4] = colorScaling * (T(inputPoint.g) - albedo[1]);
		residual[5] = colorScaling * (T(inputPoint.b) - albedo[2]);
		return true;
	}

private:
	// Input pixel that this residual is computing.
	const pcl::PointXYZRGB& inputPoint;

	const FaceModel& model;
	const Matrix4f& pose;

	// Rasterization result for this pixel.
	const PixelData& rasterizerResult;
};

struct RegularizerFunctor
{
	// TODO pass in from the outside
	const float regStrengthAlpha = 0.01f;
	const float regStrengthBeta = 0.1f;

	template <typename T>
	bool operator()(T const* alpha, T const* beta, T* residual) const {
		T factor = T(regStrengthAlpha / NUM_ALPHA_VEC);
		for (size_t i = 0; i < NUM_ALPHA_VEC; i++) {
			residual[i] = factor * alpha[i];
		}
		factor = T(regStrengthBeta / NUM_BETA_VEC);
		for (size_t i = 0; i < NUM_BETA_VEC; i++) {
			residual[NUM_ALPHA_VEC + i] = factor * beta[i];
		}
		return true;
	}
};

struct BarycentricTransform {
private:
	Vector2f offset;
	Matrix2f Ti;
public:
	BarycentricTransform(const Vector2f& s0, const Vector2f& s1, const Vector2f& s2) :
		offset(s2) {
		Matrix2f T;
		T << (s0 - s2), (s1 - s2);
		Ti = T.inverse();
	}
	Vector3f operator()(const Vector2f& v) const {
		Vector2f b;
		b = Ti * (v - offset);
		return Vector3f(b[0], b[1], 1.0f - b[0] - b[1]);
	}
};

struct RasterizerFunctor : public ceres::IterationCallback {
	RasterizerFunctor(std::vector<PixelData>& rasterizerResults, const Array2i frameSize, const double* alpha, const double* beta, const FaceModel& model, const Matrix4f& pose, const Matrix3f& intrinsics)
		: rasterizerResults(rasterizerResults), frameSize(frameSize), alpha(alpha), beta(beta), model(model), pose(pose), intrinsics(intrinsics) {}

	virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
		Matrix3Xf projectedVertices;
		Matrix4Xi vertexAlbedos;
		project(projectedVertices, vertexAlbedos);
		rasterize(projectedVertices, vertexAlbedos);
		numCalls++;
		return ceres::CallbackReturnType::SOLVER_CONTINUE;
	}

private:
	// How many times this rasterizer has been invoked so far.
	int numCalls = 0;

	const double* alpha;
	const double* beta;
	const FaceModel& model;
	const Matrix4f& pose;
	const Matrix3f& intrinsics;

	// Output of the rasterization for all pixels (barycentric coordinates and vertex indices).
	// Stores info about pixels in row-major order, so pixel (x,y) is at index (y*width + x).
	std::vector<PixelData>& rasterizerResults;
	const Array2i frameSize;

	void project(Matrix3Xf& outProjectedVertices, Matrix4Xi& outVertexAlbedos) {
		std::cout << "          Alpha: " << alpha[0] << "," << alpha[1] << "," << alpha[2] << "," << alpha[3] << ", etc." << std::endl;
		std::cout << "          Beta: " << beta[0] << "," << beta[1] << "," << beta[2] << "," << beta[3] << ", etc." << std::endl;
		std::cout << "          Rasterization: project ..." << std::flush;

		FaceParameters params = model.createDefaultParameters();
		params.alpha.head<NUM_ALPHA_VEC>() = Map<const VectorXd>(alpha, NUM_ALPHA_VEC).cast<float>();
		params.beta.head<NUM_BETA_VEC>() = Map<const VectorXd>(beta, NUM_BETA_VEC).cast<float>();

		VectorXf flatVertices = model.computeShape(params);
		Matrix3Xf worldVertices = pose.topLeftCorner<3, 3>() * Map<Matrix3Xf>(flatVertices.data(), 3, model.getNumVertices());
		worldVertices.colwise() += pose.topRightCorner<3, 1>();

		// Project to screen space.
		outProjectedVertices = intrinsics * worldVertices;
		outVertexAlbedos = model.computeColors(params);
	}

	void rasterize(const Matrix3Xf& projectedVertices, const Matrix4Xi vertexAlbedos) {
		// Reset output.
		std::fill(rasterizerResults.begin(), rasterizerResults.end(), PixelData());

		std::cout << " rasterize ..." << std::flush;

		ArrayXXf depthBuffer(frameSize.x(), frameSize.y());
		depthBuffer.setConstant(std::numeric_limits<float>::infinity());

		const Matrix3Xi& triangles = model.m_averageMesh.triangles;

		for (size_t t = 0; t < triangles.cols(); t++) {
			const auto& indices = triangles.col(t);
			Vector3f v0 = projectedVertices.col(indices(0));
			Vector3f v1 = projectedVertices.col(indices(1));
			Vector3f v2 = projectedVertices.col(indices(2));

			// Get vertices in pixel space.
			auto s0 = ((v0.head<2>() / v0.z()).array()).matrix();
			auto s1 = ((v1.head<2>() / v1.z()).array()).matrix();
			auto s2 = ((v2.head<2>() / v2.z()).array()).matrix();

			// Calculate bounds of triangle on screen.
			Array2i boundsMinPx = s0.array().min(s1.array()).min(s2.array()).cast<int>();
			Array2i boundsMaxPx = s0.array().max(s1.array()).max(s2.array()).cast<int>();
			//Array2i boundsMinPx = ((0.5f*boundsMin + 0.5f) * frameSize.cast<float>()).cast<int>();
			//Array2i boundsMaxPx = ((0.5f*boundsMax + 0.5f) * frameSize.cast<float>()).cast<int>();
			boundsMaxPx += 1;

			// Clip to actual frame buffer region.
			boundsMinPx = boundsMinPx.max(Array2i(0, 0));
			boundsMaxPx = boundsMaxPx.min(frameSize);

			BarycentricTransform bary(s0, s1, s2);

			for (int y = boundsMinPx.y(); y < boundsMaxPx.y(); y++) {
				for (int x = boundsMinPx.x(); x < boundsMaxPx.x(); x++) {
					/*Array2f pos(x, y);
					pos /= frameSize.cast<float>();
					pos = (pos - 0.5f) * 2.0f;

					Vector3f baryCoords = bary(pos.matrix());*/
					Vector3f baryCoords = bary(Vector2f(x + 0.5f, y + 0.5f));

					if ((baryCoords.array() <= 1.0f).all() && (baryCoords.array() >= 0.0f).all()) {
						float depth = baryCoords.dot(Vector3f(v0.z(), v1.z(), v2.z()));
						if (depth < depthBuffer(x, y)) {
							depthBuffer(x, y) = depth;
							PixelData& out = rasterizerResults[y * frameSize.x() + x];
							out.isValid = true;
							out.vertexIndices[0] = indices(0);
							out.vertexIndices[1] = indices(1);
							out.vertexIndices[2] = indices(2);
							out.barycentricCoordinates = baryCoords;
						}
					}
				}
			}
		}

		size_t filledPx = std::count_if(rasterizerResults.begin(), rasterizerResults.end(), [](const PixelData& px) { return px.isValid; });
		std::cout << " (valid pixels: " << filledPx << ")";

		{
			std::cout << " saving bmp ..." << std::flush;
			BMP bmp(frameSize.x(), frameSize.y());
			BMP bmpCol(frameSize.x(), frameSize.y());
			// replace infinity values in buffer with 0
			for (size_t i = 0; i < depthBuffer.size(); i++) {
				if (std::isinf(depthBuffer.data()[i]))
					depthBuffer.data()[i] = 0;
			}
			const float scale = depthBuffer.maxCoeff();
			for (int i = 0; i < depthBuffer.size(); i++) {
				Array4i depthCol;
				if (depthBuffer.data()[i] == 0) {
					depthCol = Array4i(0, 0, 30, 255);
				}
				else {
					int c = int(depthBuffer.data()[i] / scale * 255);
					depthCol = Array4i(c, c, c, 255);
				}
				bmp.data[4 * i + 2] = depthCol[0];
				bmp.data[4 * i + 1] = depthCol[1];
				bmp.data[4 * i + 0] = depthCol[2];
				bmp.data[4 * i + 3] = depthCol[3];

				auto& result = rasterizerResults[i];
				Array4f col;
				col << 0, 0, 0, 0;
				for (int v = 0; v < 3; v++) {
					float bary = result.barycentricCoordinates(v);
					auto& vcol = vertexAlbedos.col(result.vertexIndices[v]);
					col += bary * vcol.array().cast<float>();
				}

				bmpCol.data[4 * i + 2] = col[0];
				bmpCol.data[4 * i + 1] = col[1];
				bmpCol.data[4 * i + 0] = col[2];
				bmpCol.data[4 * i + 3] = col[3];
			}

			char filename[100];
			sprintf(filename, "depthmap_%d.bmp", numCalls);
			bmp.write(filename);
			sprintf(filename, "ecolmap_%d.bmp", numCalls);
			bmpCol.write(filename);
		}

		std::cout << " done!" << std::endl;
	}
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropCloudToHeadRegion(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
	const Matrix4f& pose,
	const FaceModel& model)
{
	// find average Steve's size
	pcl::PointCloud<pcl::PointXYZRGB> transformedSteve;
	pcl::transformPointCloud(*pointsToCloud(model.m_averageMesh.vertices), transformedSteve, pose);
	Vector4f min;
	Vector4f max;
	pcl::getMinMax3D(transformedSteve, min, max);

	Vector4f size = max - min;
	min = min - size / 2;
	max = max + size / 2;
	min.w() = 1;
	max.w() = 1;

	std::cout << "Crop region: " << min.transpose() << " to " << max.transpose() << std::endl;

	pcl::CropBox<pcl::PointXYZRGB> boxFilter;
	boxFilter.setMin(min);
	boxFilter.setMax(max);
	boxFilter.setInputCloud(inputCloud);
	boxFilter.setKeepOrganized(true);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
	boxFilter.filter(*out);
	return out;
}

FaceParameters optimizeParameters(FaceModel& model, const Matrix4f& pose, const Sensor& inputSensor) {
	auto croppedCloud = cropCloudToHeadRegion(inputSensor.m_cloud, pose, model);

	const uint32_t width = croppedCloud->width;
	const uint32_t height = croppedCloud->height;

	std::array<double, NUM_ALPHA_VEC> alpha{};
	std::array<double, NUM_BETA_VEC> beta{};
	std::vector<PixelData> rasterResults(width * height);

	{
		std::cout << "Saving inputsensor.bmp ..." << std::endl;
		BMP bmp(width, height);
		for (unsigned int y = 0; y < height; y++) {
			for (unsigned int x = 0; x < width; x++) {
				auto& p = (*croppedCloud)(x, y);
				if (std::isnan(p.x) || std::isnan(p.y))
					continue;
				Vector3f projectedPoint = inputSensor.m_cameraIntrinsics * Vector3f(p.x, p.y, p.z);
				auto s = projectedPoint.head<2>() / projectedPoint.z();
				int sx = int(s.x() + 0.5f);
				int sy = int(s.y() + 0.5f);

				if (sx != x || sy != y) {
					std::cout << "(" << x << "," << y << ") goes to (" << sx << "," << sy << ")" << std::endl;
				}

				if (sx >= 0 && sx < width && sy >= 0 && sy < height) {
					//int bmpIndex = (sy * width + sx);
					int bmpIndex = (y * width + x);
					bmp.data[4 * bmpIndex + 2] = p.r;
					bmp.data[4 * bmpIndex + 1] = p.g;
					bmp.data[4 * bmpIndex + 0] = p.b;
					bmp.data[4 * bmpIndex + 3] = 255;
				}
			}
		}
		bmp.write("inputsensor.bmp");
	}


	// Set up the rasterizer, which will be called once for each Ceres iteration and 
	// which updates rasterResults with the current per-pixel rendering results.
	RasterizerFunctor rasterizerCallback(rasterResults, { width, height }, alpha.data(), beta.data(), model, pose, inputSensor.m_cameraIntrinsics);
	// Initially call rasterizer once as the callback is only invoked AFTER each iteration.
	rasterizerCallback(ceres::IterationSummary());

	ceres::Problem problem;
	for (unsigned int y = 0; y < height; y += 2) {
		for (unsigned int x = 0; x < width; x += 2) {
			const pcl::PointXYZRGB& point = (*croppedCloud)(x, y);
			if (std::isnan(point.z)) {
				continue;
			}

			ceres::CostFunction* costFunc = new ceres::AutoDiffCostFunction<ResidualFunctor, NUM_DENSE_RESIDUALS, NUM_ALPHA_VEC, NUM_BETA_VEC>(
				new ResidualFunctor(point, rasterResults[y * width + x], model, pose));
			problem.AddResidualBlock(costFunc, NULL, alpha.data(), beta.data());
		}
	}

	// Add regularization error term.
	ceres::CostFunction* regFunc = new ceres::AutoDiffCostFunction<RegularizerFunctor, NUM_ALPHA_VEC+NUM_BETA_VEC, NUM_ALPHA_VEC, NUM_BETA_VEC>(new RegularizerFunctor());
	problem.AddResidualBlock(regFunc, NULL, alpha.data(), beta.data());

	std::cout << "Cost function has " << problem.NumResidualBlocks() << " residual blocks." << std::endl;

	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.update_state_every_iteration = true;
	options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
	options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
	options.initial_trust_region_radius = 0.01;
	options.max_trust_region_radius = 0.15;
	options.callbacks.push_back(&rasterizerCallback);
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.FullReport() << std::endl;
	std::cout << "Some final values of alpha: " << Map<VectorXd>(alpha.data(), 10) << std::endl;
	std::cout << "Some final values of beta: " << Map<VectorXd>(beta.data(), 10) << std::endl;

	FaceParameters params = model.createDefaultParameters();
	params.alpha.head<NUM_ALPHA_VEC>() = Map<const VectorXd>(alpha.data(), NUM_ALPHA_VEC).cast<float>();
	params.beta.head<NUM_BETA_VEC>() = Map<const VectorXd>(beta.data(), NUM_BETA_VEC).cast<float>();

	return params;
}
