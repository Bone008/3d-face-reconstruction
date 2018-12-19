#pragma once
#include "Mesh.h"

struct FaceParameters {
	Eigen::VectorXf alpha;
	// ... later: albedo, lighting, expression ...
};

class FaceModel
{
public:
	FaceModel(const std::string& baseDir);

	// 3D positions of 5 feature points used for coarse alignment.
	std::vector<Eigen::Vector3f> m_averageFeaturePoints;

	// Vertex positions of the average face. Packed shape (3 * numVertices).
	Mesh m_averageShapeMesh;

	// Orthogonal basis for the shape parameters alpha. Shape (3 * numVertices, numEigenVec)
	Eigen::MatrixXf m_shapeBasis;
	// Orthogonal basis for the expression parameters delta. Shape (3 * numVertices, numExprVec)
	Eigen::MatrixXf m_expressionBasis;
	

	// Computes the vertex positions of a face based on a set of parameters.
	Eigen::VectorXf computeShape(const FaceParameters& params);

	unsigned int getNumVertices() { return m_averageShapeMesh.vertices.rows() / 3; }
	unsigned int getNumEigenVec() { return m_shapeBasis.cols(); }
	unsigned int getNumExprVec() { return m_expressionBasis.cols(); }

private:
	const Mesh loadOFF(const std::string & filename);
	std::vector<float> loadBinaryVector(const std::string &filename);
};

