#pragma once
#include "Mesh.h"

struct FaceParameters {
	// Shape parameters expressed as multiples of the standard deviation.
	Eigen::VectorXf alpha;
	// Albedo parameters.
	Eigen::VectorXf beta;
	// Illumination parameters.
	Eigen::VectorXf gamma;
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
    Eigen::Matrix3Xf computeNormals(const Eigen::VectorXf& vertices) const;

	FaceParameters computeShapeAttribute(const FaceParameters& params, float age, float weight, float gender) const;

	inline FaceParameters createDefaultParameters() const {
		FaceParameters params;
		params.alpha.resize(getNumEigenVec());
		params.alpha.setZero();
		params.beta.resize(getNumEigenVec());
		params.beta.setZero();
		params.gamma.resize(9);
		//params.gamma.setConstant(1.0f);

		params.gamma(0) = .79f;
		params.gamma(1) = .39f;
		params.gamma(2) = -.34f;
		params.gamma(3) = .79f;
		params.gamma(4) = .79f;

		/*
		 * _L00 ("L00", Vector) = (.79, .44, .54, 1)
        _L1N1 ("L1-1", Vector) = (.39, .35, .60, 1)
        _L10 ("L10", Vector) = (-.34, -.18, -.27, 1)
        _L11 ("L11", Vector) = (-.29, -.06, .01, 1)
        _L2N2 ("L2-2", Vector) = (-.11, -.05, -.12, 1)
        _L2N1 ("L2-1", Vector) = (-.26, -.22, -.47, 1)
        _L20 ("L20", Vector) = (-.16, -.09, -.15, 1)
        _L21 ("L21", Vector) = (.56, .21, .14, 1)
        _L22 ("L22", Vector) = (.21, -.05, -.30, 1)
		 */

		return params;
	}

	unsigned int getNumVertices() const { return m_averageMesh.getNumVertices(); }
	unsigned int getNumEigenVec() const { return m_shapeBasis.cols(); }
	unsigned int getNumExprVec() const { return m_expressionBasis.cols(); }

private:
	const Mesh loadOFF(const std::string & filename) const;
	std::vector<float> loadBinaryVector(const std::string &filename) const;
};

