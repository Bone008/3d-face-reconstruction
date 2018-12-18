#pragma once

struct FaceParameters {
	Eigen::VectorXf alpha;
	// ... later: albedo, lighting, expression ...
};

class FaceModel
{
public:
	FaceModel(const std::string& baseDir);

	Eigen::VectorXf m_averageShape;
	Eigen::MatrixXf m_shapeBasis;

	// Computes the vertex positions of a face based on a set of parameters.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr computeShape(const FaceParameters& params);
};

