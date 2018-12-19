#include "stdafx.h"
#include "FaceModel.h"

const std::string filenameAverageMesh = "averageMesh.off";
const std::string filenameAverageMeshFeaturePoints = "averageMesh_features.points";
const std::string filenameBasisShape = "ShapeBasis.matrix";
const std::string filenameBasisExpression = "ExpressionBasis.matrix";
const std::string filenameStdDevShape = "StandardDeviationShape.vec";
const std::string filenameStdDevExpression = "StandardDeviationExpression.vec";

FaceModel::FaceModel(const std::string& baseDir) {
	// load average shape
	m_averageShapeMesh = loadOFF(baseDir + filenameAverageMesh);
	m_averageShapeMesh.vertices /= 1000000.0f;
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
	return m_averageShapeMesh.vertices + m_shapeBasis * params.alpha;
}

// Utility function to primitively load a file in the STOFF format.
const Mesh FaceModel::loadOFF(const std::string& filename) {
	std::ifstream in(filename, std::ifstream::in);
	if (!in) {
		std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
		exit(1);
	}

	std::string line;
	std::getline(in, line); // Header, should be STOFF
	std::getline(in, line); // contains number of vertices and triangles
	std::istringstream headerStream(line);
	int nVertices, nTriangles;
	headerStream >> nVertices >> nTriangles;

	Mesh mesh;
	mesh.vertices.resize(3 * nVertices);
	mesh.vertexColors.resize(4, nVertices);
	mesh.triangles.resize(3, nTriangles);

	std::cout << "  vertices ..." << std::endl;
	float x, y, z;
	int r, g, b, a;
	for (int i = 0; i < nVertices; i++) {
		std::getline(in, line);
		std::istringstream lineStream(line);
		lineStream >> x >> y >> z >> r >> g >> b >> a;
		mesh.vertices(3 * i + 0) = x;
		mesh.vertices(3 * i + 1) = y;
		mesh.vertices(3 * i + 2) = z;
		mesh.vertexColors.col(i) << r, g, b, a;
	}

	std::cout << "  triangles ..." << std::endl;
	int count, v1, v2, v3;
	for (int i = 0; i < nTriangles; i++) {
		std::getline(in, line);
		std::istringstream lineStream(line);
		lineStream >> count >> v1 >> v2 >> v3;

		if (count != 3) {
			std::cerr << "WARNING: Can only process triangles, found face with " << count << " vertices while reading " << filename << std::endl;
			v1 = v2 = v3 = 0;
		}

		mesh.triangles.col(i) << v1, v2, v3;
	}

	in.close();

	return mesh;
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
