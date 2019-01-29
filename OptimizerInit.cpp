#include "stdafx.h"
#include "OptimizerInit.h"
#include "FaceModel.h"
#include "utils.h"
#include <pcl/kdtree/kdtree_flann.h>

using namespace Eigen;

const unsigned int NUM_LINEAR_ITERATIONS = 5;
const double MAX_CORRESPONDENCE_DISTANCE = 0.01;
const int NUM_LINEAR_ALPHA_DIM = 40;

struct Match {
	// Index of the matched point in the model cloud.
	size_t idModel;
	// Index of the matched point in the input cloud.
	size_t idInput;
};

std::vector<Match> findCorrespondences(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr modelCloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr localInputCloud) {
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdTree;
	kdTree.setInputCloud(localInputCloud);

	std::vector<Match> matches;

	int pointsWithMatches = 0;
	float angleSum = 0;
	for (size_t i = 0; i < modelCloud->points.size(); i++) {
		const pcl::PointXYZRGBNormal& modelPoint = modelCloud->points[i];

		// Skip vertices with invalid normals.
		if (!modelPoint.getNormalVector3fMap().allFinite()) {
			continue;
		}

		std::vector<int> candidateIndices(1);
		std::vector<float> candidateSqrDistances(1);
		int result = kdTree.nearestKSearch(modelPoint, 1, candidateIndices, candidateSqrDistances);

		if (result == 0) {
			continue;
		}

		const auto& matchedPoint = localInputCloud->points[candidateIndices[0]];
		// Skip matches with invalid normals.
		if (!matchedPoint.getNormalVector3fMap().allFinite()) {
			continue;
		}

		float normalAngle = acosf(modelPoint.getNormalVector3fMap().dot(matchedPoint.getNormalVector3fMap()));
		float normalAngleDeg = normalAngle * 180.0 / M_PI;
		angleSum += normalAngleDeg;
		pointsWithMatches++;

		// Skip matches based on angle and max distance.
		if (normalAngleDeg > 20.0 || candidateSqrDistances[0] > MAX_CORRESPONDENCE_DISTANCE * MAX_CORRESPONDENCE_DISTANCE) {
			continue;
		}

		matches.push_back({ i, (size_t)candidateIndices[0] });
	}

	std::cout << "  points with matches: " << matches.size() << " / " << pointsWithMatches << " / " << modelCloud->points.size() << std::endl;
	std::cout << "  average angle: " << (angleSum / pointsWithMatches) << std::endl;
	return matches;
}

VectorXf linearOptimizeShape(const FaceModel& model, const Eigen::Matrix4f& pose, pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr inputCloud, const VectorXf& initialAlpha) {
	VectorXf alpha = initialAlpha;

	// Transform input cloud to model space. In the rest of the project, the model
	// is transformed to the input, but doing it this way makes the linear equation simpler later.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr localInputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::transformPointCloudWithNormals(*inputCloud, *localInputCloud, pose.inverse());

	VectorXf modelVertices = model.computeShape(alpha);
	Matrix3Xf modelNormals = model.computeNormals(modelVertices);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelCloud = pointsToCloud(modelVertices, modelNormals);

	std::vector<Match> matches = findCorrespondences(modelCloud, localInputCloud);
	if (matches.empty()) {
		std::cout << "  Skipped because no good matches were found!" << std::endl;
		return alpha;
	}

	// We want to solve the linear equation:
	//   ShapeBasis * sigma * alpha = InputPoints - AverageShapePoints
	//              A       *   x   =             b
	// where only rows with valid matches and only NUM_LINEAR_ALPHA_DIM columns of ShapeBasis are used.
	MatrixXf A(3 * matches.size(), NUM_LINEAR_ALPHA_DIM);
	VectorXf b(3 * matches.size());

	Array<float, 1, NUM_LINEAR_ALPHA_DIM> standardDeviationsArr = model.m_shapeStd.head<NUM_LINEAR_ALPHA_DIM>().transpose().array();
	for (size_t i = 0; i < matches.size(); i++) {
		Matrix<float, 3, NUM_LINEAR_ALPHA_DIM> basisBlock = model.m_shapeBasis.block(3 * matches[i].idModel, 0, 3, NUM_LINEAR_ALPHA_DIM);
		basisBlock.array().rowwise() *= standardDeviationsArr;
		A.middleRows<3>(3 * i) = basisBlock;

		const auto& inputPoint = localInputCloud->points[matches[i].idInput].getVector3fMap();
		const auto& averagePoint = model.m_averageMesh.vertices.segment<3>(3 * matches[i].idModel);
		b.segment<3>(3 * i) = inputPoint - averagePoint;
	}

	std::cout << "  Running linear solver ... ";
	VectorXf leastSquaresAlpha = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
	std::cout << "done!" << std::endl;

	std::cout << "  initial error L2 norm: " << b.norm() << std::endl;
	std::cout << "    final error L2 norm: " << (A * leastSquaresAlpha - b).norm() << std::endl;

	alpha.head<NUM_LINEAR_ALPHA_DIM>() = leastSquaresAlpha;
	return alpha;
}

VectorXf initializeShapeParameters(const FaceModel& model, const Eigen::Matrix4f& pose, pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr inputCloud) {
	std::cout << "Param initialization ..." << std::endl;
	VectorXf alpha = VectorXf::Zero(model.getNumEigenVec());

	for (unsigned int i = 0; i < NUM_LINEAR_ITERATIONS; i++) {
		alpha = linearOptimizeShape(model, pose, inputCloud, alpha);
		std::cout << "  some alphas (iter " << (i+1) << "): " << alpha.head<5>().transpose() << std::endl;
	}

	return alpha;
}
