#pragma once
#include "Mesh.h"

struct FaceParameters {
	// Shape parameters expressed as multiples of the standard deviation.
	Eigen::VectorXf alpha;
	// Albedo parameters.
	Eigen::VectorXf beta;
	// ... later: lighting, expression ...
};

class FaceModel
{
public:
	FaceModel(const std::string& baseDir);

	// 3D positions of 5 feature points used for coarse alignment.
	std::vector<Eigen::Vector3f> m_averageFeaturePoints;

	// Vertex positions of the average face. Packed shape (3 * numVertices).
	// TODO remove "shape" from name as it is not accurate
	Mesh m_averageMesh;

	// Orthogonal basis for the shape parameters alpha. Shape (3 * numVertices, numEigenVec)
	Eigen::MatrixXf m_shapeBasis;
	// Orthogonal basis for the albedo parameters beta. Shape (3 * numVertices, numEigenVec)
	Eigen::MatrixXf m_albedoBasis;
	// Orthogonal basis for the expression parameters delta. Shape (3 * numVertices, numExprVec)
	Eigen::MatrixXf m_expressionBasis;

	// Standard deviation of the shape parameters alpha. Shape (numEigenVec)
	Eigen::VectorXf m_shapeStd;
	// Standard deviation of the albedo parameters beta. Shape (numEigenVec)
	Eigen::VectorXf m_albedoStd;
	// Standard deviation of the expression parameters delta . Shape (numExprVec)
	Eigen::VectorXf m_expressionStd;


	// Computes the vertex positions based on a set of parameters.
	Eigen::VectorXf computeShape(const FaceParameters& params) const;
	// Computes the vertex colors based on a set of parameters.
	Eigen::Matrix4Xi computeColors(const FaceParameters& params) const;

    // Computes the vertex positions based on a set of parameters.
    Eigen::Matrix3Xf computeNormals(const FaceParameters& params, const Eigen::Matrix3Xf& worldVertices) const;

	FaceParameters computeShapeAttribute(const FaceParameters& params, float age, float weight, float gender) const;

	inline FaceParameters createDefaultParameters() const {
		FaceParameters params;
		params.alpha.resize(getNumEigenVec());
		params.alpha.setZero();
		params.beta.resize(getNumEigenVec());
		params.beta.setZero();
		return params;
	}

	unsigned int getNumVertices() const { return m_averageMesh.getNumVertices(); }
	unsigned int getNumEigenVec() const { return m_shapeBasis.cols(); }
	unsigned int getNumExprVec() const { return m_expressionBasis.cols(); }

private:
	const Mesh loadOFF(const std::string & filename) const;
	std::vector<float> loadBinaryVector(const std::string &filename) const;
};

