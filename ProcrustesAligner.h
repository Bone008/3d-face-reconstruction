#pragma once

class ProcrustesAligner {
public:
	Eigen::Matrix4f estimatePose(const std::vector<Eigen::Vector3f>& sourcePoints, const std::vector<Eigen::Vector3f>& targetPoints);

private:
	Eigen::Vector3f computeMean(const std::vector<Eigen::Vector3f>& points);

	float computeScale(std::vector<Eigen::Vector3f>& sourcePoints, const Eigen::Vector3f& sourceMean,
										  const std::vector<Eigen::Vector3f>& targetPoints, const Eigen::Vector3f& targetMean);

		Eigen::Matrix3f estimateRotation(
		const std::vector<Eigen::Vector3f>& sourcePoints,
		const Eigen::Vector3f& sourceMean,
		const std::vector<Eigen::Vector3f>& targetPoints,
		const Eigen::Vector3f& targetMean);
	Eigen::Vector3f computeTranslation(const Eigen::Vector3f& sourceMean, const Eigen::Vector3f& targetMean);
};
