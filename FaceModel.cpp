#include "stdafx.h"
#include "FaceModel.h"

const std::string filenameAverageMesh = "averageMesh.off";
const std::string filenameAverageMeshFeaturePoints = "averageMesh_features.points";
const std::string filenameBasisShape = "ShapeBasis.matrix";
const std::string filenameBasisExpression = "ExpressionBasis.matrix";
const std::string filenameStdDevShape = "StandardDeviationShape.vec";
const std::string filenameStdDevExpression = "StandardDeviationExpression.vec";

FaceModel::FaceModel(const std::string& baseDir)
{
	// load average shape
	m_averageShape = loadOFF(baseDir + filenameAverageMesh);
	m_averageShape /= 1000000.0f;
	// load average shape feature points
	FeaturePointExtractor averageFeatureExtractor(baseDir + filenameAverageMeshFeaturePoints, nullptr);
	m_averageFeaturePoints = averageFeatureExtractor.m_points;

	unsigned int nVertices = getNumVertices();

	// load shape basis
	std::vector<float> shapeBasisRaw = loadBinaryVector(baseDir + filenameBasisShape);
	unsigned int nEigenVec = shapeBasisRaw.size() / (4 * nVertices);
	// convert to matrix
	Eigen::Map<Eigen::MatrixXf> shapeBasisVec4(shapeBasisRaw.data(), 4 * nVertices, nEigenVec);

	// discard 4th vertex coordinate
	m_shapeBasis.resize(3 * nVertices, nEigenVec);
	for (int i = 0; i < nVertices; i++) {
		m_shapeBasis.block(3 * i, 0, 3, nEigenVec) = shapeBasisVec4.block(4 * i, 0, 3, nEigenVec);
	}

	std::vector<float> expressionBasisRaw = loadBinaryVector(baseDir + filenameBasisExpression);
	unsigned int nExpr = expressionBasisRaw.size() / (4 * nVertices);
	// convert to matrix
	Eigen::Map<Eigen::MatrixXf> expressionBasisVec4(expressionBasisRaw.data(), 4 * nVertices, nExpr);

	// discard 4th vertex coordinate
	m_expressionBasis.resize(3 * nVertices, nExpr);
	for (int i = 0; i < nVertices; i++) {
		m_expressionBasis.block(3 * i, 0, 3, nExpr) = expressionBasisVec4.block(4 * i, 0, 3, nExpr);
	}

	// not needed yet
	/*
	auto shapeDevRaw = new float[nEigenVec];
	LoadVector(filenameStdDevShape, shapeDevRaw, nEigenVec);

	auto expressionDevRaw = new float[nExpr];
	LoadVector(filenameStdDevExpression, expressionDevRaw, nExpr);
	*/
}

Eigen::VectorXf FaceModel::computeShape(const FaceParameters& params)
{
	return m_averageShape + m_shapeBasis * params.alpha;
}

// Utility function to primitively load a file in the STOFF format.
const Eigen::VectorXf FaceModel::loadOFF(const std::string& filename) {
	std::ifstream in(filename, std::ifstream::in);
	if (!in) {
		std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
		exit(1);
	}

	std::string dummy;
	in >> dummy; // Header, should be STOFF
	int nVertices;
	in >> nVertices;

	Eigen::VectorXf mat(3 * nVertices);
	float x, y, z;
	std::string line;
	for (int i = 0; i < nVertices; i++) {
		std::getline(in, line);
		std::istringstream lineStream(line);
		lineStream >> x >> y >> z;
		mat(3 * i + 0) = x;
		mat(3 * i + 1) = y;
		mat(3 * i + 2) = z;
	}

	in.close();

	return mat;
}

std::vector<float> FaceModel::loadBinaryVector(const std::string &filename) {
	std::ifstream in(filename, std::ifstream::in | std::ifstream::binary);
	if (!in) {
		std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
		exit(1);
	}
	unsigned int numberOfEntries;
	in.read((char *)&numberOfEntries, sizeof(unsigned int));

	std::vector<float> result(numberOfEntries);
	in.read((char*)result.data(), numberOfEntries * sizeof(float));

	in.close();
	return result;
}
