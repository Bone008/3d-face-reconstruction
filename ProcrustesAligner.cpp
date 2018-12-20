#include "stdafx.h"
#include "ProcrustesAligner.h"

using namespace Eigen;

Matrix4f ProcrustesAligner::estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
	assert(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

	// We estimate the pose between source and target points using Procrustes algorithm.
	// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
	// from source points to target points.

	auto sourceMean = computeMean(sourcePoints);
	auto targetMean = computeMean(targetPoints);

	Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
	Vector3f translation = computeTranslation(sourceMean, targetMean);

	// To apply the pose to point x on shape X in the case of Procrustes, we execute:
	// 1. Translation of a point to the shape Y: x' = x + t
	// 2. Rotation of the point around the mean of shape Y: 
	//    y = R (x' - yMean) + yMean = R (x + t - yMean) + yMean = R x + (R t - R yMean + yMean)

	Matrix4f estimatedPose = Matrix4f::Identity();
	estimatedPose.block(0, 0, 3, 3) = rotation;
	estimatedPose.block(0, 3, 3, 1) = rotation * translation - rotation * targetMean + targetMean;

	return estimatedPose;
}


Vector3f ProcrustesAligner::computeMean(const std::vector<Vector3f>& points) {
	// Compute the mean of input points.

	Vector3f result = Vector3f::Zero();
	for (unsigned int i = 0; i < points.size(); i++) {
		result += points[i];
	}

	return result / points.size();
}

Matrix3f ProcrustesAligner::estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
	// Estimate the rotation from source to target points, following the Procrustes algorithm.
	// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
	// Important: The covariance matrices should contain mean-centered source/target points.

	// convert source points to matrix
	MatrixXf sourcePointsMatrix(sourcePoints.size(), 3);
	for (int y = 0; y < sourcePoints.size(); y++) {
		sourcePointsMatrix.row(y) = sourcePoints[y] - sourceMean;
	}

	// convert target points to matrix
	MatrixXf targetPointsMatrix(targetPoints.size(), 3);
	for (int y = 0; y < targetPoints.size(); y++) {
		targetPointsMatrix.row(y) = targetPoints[y] - targetMean;
	}

	// split into U, S and V
	JacobiSVD<MatrixXf> svd(targetPointsMatrix.transpose() * sourcePointsMatrix, ComputeThinU | ComputeThinV);

	return svd.matrixU() * svd.matrixV().transpose();
}

Vector3f ProcrustesAligner::computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean) {
	// Compute the translation vector from source to target points.

	return targetMean - sourceMean;
}