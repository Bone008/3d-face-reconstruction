#include "stdafx.h"
#include <pcl/filters/crop_box.h>
#include "Optimizer.h"
#include "Rasterizer.h"
#include "BMP.h"
#include "utils.h"
#include "Settings.h"

using namespace Eigen;

// Constant to allow better compile-time optimization.
// If this is smaller than the number of actual eigen vectors (160),
// only the first ones will be optimized over.
const unsigned int NUM_ALPHA_VEC = 160;
const unsigned int NUM_BETA_VEC = 80;

const unsigned int NUM_DENSE_RESIDUALS = 4 + 3;

struct ResidualFunctor {
	// x is the source (pos mesh), y is the target (input cloud)
	ResidualFunctor(const pcl::PointXYZRGBNormal& inputPoint, const PixelData& rasterizerResult, const FaceModel& model, const Matrix4f& pose, const Matrix3f& intrinsics, const Vector3f& colorDelta)
		: inputPoint(inputPoint), rasterizerResult(rasterizerResult), model(model), pose(pose), intrinsics(intrinsics), colorDelta(colorDelta) {}

	template <typename T>
	bool operator()(T const* alpha, T const* beta, T* residual) const {
		typedef Matrix<T, 2, 1> Vector2T;
		typedef Matrix<T, 3, 1> Vector3T;
		typedef Matrix<T, 2, 2> Matrix2T;
		typedef Matrix<T, 3, 3> Matrix3T;

		if (!rasterizerResult.isValid) {
			// Skip pixels where Steve isn't rendered into.
			std::fill(residual, residual + NUM_DENSE_RESIDUALS, T(0));
			return true;
		}

		Vector3T vertexWorldPositions[3];
		Vector3T vertexAlbedos[3];
		Vector2T vertexScreenPositions[3];

		// For each vertex that is part of the triangle at this pixel.
		for (int i = 0; i < 3; i++) {
			int vertexIndex = rasterizerResult.vertexIndices[i];

			// Albedo of average face (ignore alpha).
			vertexAlbedos[i] = model.m_averageMesh.vertexColors.col(vertexIndex).head<3>().cast<T>();
			// Apply beta to albedo.
			for (int j = 0; j < NUM_BETA_VEC; j++) {
				T std = T(model.m_albedoStd(j));
				vertexAlbedos[i] += model.m_albedoBasis.block(3 * vertexIndex, j, 3, 1).cast<T>() * std * beta[j];
			}

			// Vertex position of average face.
			Vector3T pos = model.m_averageMesh.vertices.segment(3 * vertexIndex, 3).cast<T>();
			// Displace by applying alpha.
			for (int j = 0; j < NUM_ALPHA_VEC; j++) {
				T std = T(model.m_shapeStd(j));
				pos += model.m_shapeBasis.block(3 * vertexIndex, j, 3, 1).cast<T>() * std * alpha[j];
			}

			// Transform to world space.
			vertexWorldPositions[i] = pose.topLeftCorner<3, 3>().cast<T>() * pos + pose.topRightCorner<3, 1>().cast<T>();
			// Transform to screen space.
			Vector3T projectedPos = intrinsics.cast<T>() * vertexWorldPositions[i];
			vertexScreenPositions[i] = ((projectedPos.template head<2>() / projectedPos.z()).array()).matrix();
		}

		// Compute barycentric coordinates from screen positions;
		Matrix2T mT;
		mT << (vertexScreenPositions[0] - vertexScreenPositions[2]),
			(vertexScreenPositions[1] - vertexScreenPositions[2]);
		Matrix2T mTi = mT.inverse();

		Vector2T b = mTi * (rasterizerResult.pixelCenter.cast<T>() - vertexScreenPositions[2]);
		T barycentricCoordinates[] = {
			b(0),
			b(1),
			T(1.0f) - b(0) - b(1)
		};

		// Interpolate final values for this pixel.
		Vector3T worldPos = Vector3T::Zero();
		Vector3T albedo = Vector3T::Zero();
		for (int i = 0; i < 3; i++) {
			worldPos += barycentricCoordinates[i] * vertexWorldPositions[i];
			albedo += barycentricCoordinates[i] * vertexAlbedos[i];
		}

		Vector3T inputPos = Vector3T(T(inputPoint.x), T(inputPoint.y), T(inputPoint.z));
		Vector3T pointToPointDist = inputPos - worldPos;
		residual[0] = pointToPointDist(0);
		residual[1] = pointToPointDist(1);
		residual[2] = pointToPointDist(2);

		// TODO: point to plane distance, but for this we need normals
		residual[6] = pointToPointDist(0)*T(inputPoint.normal_x) + pointToPointDist(1)*T(inputPoint.normal_y) + pointToPointDist(2)*T(inputPoint.normal_z);

		Vector3T inputCol = Vector3T(T(inputPoint.r), T(inputPoint.g), T(inputPoint.b));
		T colorScaling = T(1.f / 255.f);
		Vector3T colorDist = (inputCol - albedo + colorDelta.cast<T>()) / T(255.0f);
		residual[3] = colorDist(0);
		residual[4] = colorDist(1);
		residual[5] = colorDist(2);
		return true;
	}

private:
	// Input pixel that this residual is computing.
	const pcl::PointXYZRGBNormal& inputPoint;

	const FaceModel& model;
	const Matrix4f& pose;
	const Matrix3f& intrinsics;
	const Vector3f& colorDelta;

	// Rasterization result for this pixel.
	const PixelData& rasterizerResult;
};

struct RegularizerFunctor
{
	template <typename T>
	bool operator()(T const* alpha, T const* beta, T* residual) const {
		T factor = T(gSettings.regStrengthAlpha / NUM_ALPHA_VEC);
		for (size_t i = 0; i < NUM_ALPHA_VEC; i++) {
			residual[i] = factor * alpha[i];
		}
		factor = T(gSettings.regStrengthBeta / NUM_BETA_VEC);
		for (size_t i = 0; i < NUM_BETA_VEC; i++) {
			residual[NUM_ALPHA_VEC + i] = factor * beta[i];
		}
		return true;
	}
};

struct RasterizerFunctor : public ceres::IterationCallback {
	RasterizerFunctor(Rasterizer& rasterizer, const double* alpha, const double* beta)
		: alpha(alpha), beta(beta), rasterizer(rasterizer) {}

	virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
		FaceParameters params = rasterizer.model.createDefaultParameters();
		params.alpha.head<NUM_ALPHA_VEC>() = Map<const VectorXd>(alpha, NUM_ALPHA_VEC).cast<float>();
		params.beta.head<NUM_BETA_VEC>() = Map<const VectorXd>(beta, NUM_BETA_VEC).cast<float>();

		rasterizer.compute(params);
		return ceres::CallbackReturnType::SOLVER_CONTINUE;
	}

private:
	Rasterizer& rasterizer;
	const double* alpha;
	const double* beta;
};

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cropCloudToHeadRegion(
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
	// load point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud = out;
	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::COVARIANCE_MATRIX);
	ne.setNormalSmoothingSize(10.0f);
	ne.setDepthDependentSmoothing(true);
	ne.compute(*normals);
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dst(new pcl::PointCloud<pcl::PointXYZRGBNormal>); // To be created
	std::cout << "1" << endl;
	// Initialization part
	dst->width = out->width;
	std::cout << "2" << endl;
	dst->height = out->height;
	std::cout << "3" << endl;
	dst->is_dense = true;
	std::cout << "4" << endl;
	dst->points.resize(dst->width * dst->height);
	std::cout << "intialisation pointNormal" << endl;
	// Assignment part
	for (int i = 0; i < normals->points.size(); i++)
	{
		dst->points.at(i).x = out->points.at(i).x;
		dst->points.at(i).y = out->points.at(i).y;
		dst->points.at(i).z = out->points.at(i).z;

		dst->points.at(i).r = out->points.at(i).r;
		dst->points.at(i).g = out->points.at(i).g;
		dst->points.at(i).b = out->points.at(i).b;

		// cloud_normals -> Which you have already have; generated using pcl example code 

		dst->points.at(i).curvature = normals->points[i].curvature;

		dst->points.at(i).normal_x = normals->points[i].normal_x;
		dst->points.at(i).normal_y = normals->points[i].normal_y;
		dst->points.at(i).normal_z = normals->points[i].normal_z;
	}
	return dst;
}

FaceParameters optimizeParameters(FaceModel& model, const Matrix4f& pose, const Sensor& inputSensor) {
	auto croppedCloud = cropCloudToHeadRegion(inputSensor.m_cloud, pose, model);

	const uint32_t width = croppedCloud->width;
	const uint32_t height = croppedCloud->height;

	std::array<double, NUM_ALPHA_VEC> alpha{};
	std::array<double, NUM_BETA_VEC> beta{};

	{
		std::cout << "Saving inputsensor.bmp ..." << std::endl;
		int warnCount = 0;
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

				if ((sx != x || sy != y) && warnCount++ < 10) {
					std::cout << "    (" << x << "," << y << ") goes to (" << sx << "," << sy << ")" << std::endl;
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
	Rasterizer rasterizer({ width, height }, model, pose, inputSensor.m_cameraIntrinsics);
	RasterizerFunctor rasterizerCallback(rasterizer, alpha.data(), beta.data());
	// Initially call rasterizer once as the callback is only invoked AFTER each iteration.
	rasterizerCallback(ceres::IterationSummary());

	pcl::PointXYZRGBNormal centroid;
	pcl::computeCentroid(*croppedCloud, centroid);
	Vector3f inputAverageCol = Vector3f(centroid.r, centroid.g, centroid.b);
	Vector3f modelAverageCol = rasterizer.getAverageColor();
	// Contains the RGB difference due to lighting from the input face to the synthetic face.
	Vector3f colorDelta = modelAverageCol - inputAverageCol;

	std::cout << "| input average: " << inputAverageCol.transpose() << std::endl;
	std::cout << "| model average: " << modelAverageCol.transpose() << std::endl;
	std::cout << "|        delta: " << colorDelta.transpose() << std::endl;


	ceres::Problem problem;
	unsigned int stride = gSettings.optimizationStride;
	for (unsigned int y = 0; y < height; y += stride) {
		for (unsigned int x = 0; x < width; x += stride) {
			const pcl::PointXYZRGBNormal& point = (*croppedCloud)(x, y);
			if (std::isnan(point.z)) {
				continue;
			}
			if (std::isnan(point.normal_x)) {
				continue;
			}
			ceres::CostFunction* costFunc = new ceres::AutoDiffCostFunction<ResidualFunctor, NUM_DENSE_RESIDUALS, NUM_ALPHA_VEC, NUM_BETA_VEC>(
				new ResidualFunctor(point, rasterizer.pixelResults[y * width + x], model, pose, inputSensor.m_cameraIntrinsics, colorDelta));
			problem.AddResidualBlock(costFunc, NULL, alpha.data(), beta.data());
		}
	}

	// Add regularization error term.
	ceres::CostFunction* regFunc = new ceres::AutoDiffCostFunction<RegularizerFunctor, NUM_ALPHA_VEC + NUM_BETA_VEC, NUM_ALPHA_VEC, NUM_BETA_VEC>(new RegularizerFunctor());
	problem.AddResidualBlock(regFunc, NULL, alpha.data(), beta.data());

	std::cout << "Cost function has " << problem.NumResidualBlocks() << " residual blocks." << std::endl;

	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.update_state_every_iteration = true;
	options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
	options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
	options.initial_trust_region_radius = gSettings.initialStepSize;
	options.max_trust_region_radius = gSettings.maxStepSize;
	options.callbacks.push_back(&rasterizerCallback);
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.FullReport() << std::endl;

	FaceParameters params = model.createDefaultParameters();
	params.alpha.head<NUM_ALPHA_VEC>() = Map<const VectorXd>(alpha.data(), NUM_ALPHA_VEC).cast<float>();
	params.beta.head<NUM_BETA_VEC>() = Map<const VectorXd>(beta.data(), NUM_BETA_VEC).cast<float>();

	std::cout << "Some final values of alpha: " << params.alpha.head<10>().transpose() << std::endl;
	std::cout << "Some final values of beta: " << params.beta.head<10>().transpose() << std::endl;

	return params;
}
