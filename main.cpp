#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Eigen>
#include "main.h"


const Eigen::MatrixX3f LoadOFF(std::string& filename) {
	std::ifstream in(filename, std::ifstream::in);
	if (!in)
	{
		std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
		throw;
	}

	std::string dummy;
	in >> dummy; // Header
	int nVertices, nFaces;
	in >> nVertices, nFaces;

	Eigen::MatrixX3f mat(nVertices, 3);
	float x, y, z;
	std::string line;
	for(int i=0; i< nVertices; i++)
	{
		std::getline(in, line);
		std::istringstream lineStream(line);
		lineStream >> x >> y >> z;
		mat.row(i) << x, y, z;
	}

	in.close();

	return mat;
}

void WriteOFF(const std::string& filename, const std::vector<Eigen::Vector3f>& data) {
	std::ofstream out(filename);
	for (const auto& v : data) {
		out << v(0) << " " << v(1) << " " << v(2) << std::endl;
	}
	out.close();
}

void LoadVector(const std::string &filename, float *res, unsigned int length)
{
    std::ifstream in(filename, std::ifstream::in | std::ifstream::binary);
    if (!in)
    {
        std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
        return;
    }
    unsigned int numberOfEntries;
    in.read((char*)&numberOfEntries, sizeof(unsigned int));
    if (length == 0) length = numberOfEntries;
    in.read((char*)(res), length * sizeof(float));

    in.close();
}

// FIXME i am not used
float* LoadEigenvectors(const std::string &filename, unsigned int components, unsigned int numberOfEigenvectors)
{
    auto *res = new float[components*numberOfEigenvectors];

    for (unsigned int i = 0; i < numberOfEigenvectors; i++)
    {
        std::stringstream ss;
        ss << filename << i << ".vec";

        LoadVector(ss.str().c_str(), &(res[components*i]), 0);
    }

    return res;
}

int main(int argc, char **argv) {
    // load face model
    int nVertices = 53490;
    int NumberOfExpressions = 76;
    int NumberOfEigenvectors = 160;

    std::string filenameBaseModel = "data/MorphableModel/";
	std::string filenameAverageMesh = filenameBaseModel + "averageMesh.off";
	std::string filenameBasisShape = filenameBaseModel + "ShapeBasis.matrix";
    std::string filenameBasisExpression = filenameBaseModel + "ExpressionBasis.matrix";
    std::string filenameStdDevShape = filenameBaseModel + "StandardDeviationShape.vec";
    std::string filenameStdDevExpression = filenameBaseModel + "StandardDeviationExpression.vec";

    // the next two floats were originally float4
    auto shapeBasisCPU = new Eigen::Vector4f[nVertices * NumberOfEigenvectors];
    auto expressionBasisCPU = new float[4 * nVertices * NumberOfExpressions];
    LoadVector(filenameBasisShape, (float *) shapeBasisCPU, 4 * nVertices * NumberOfEigenvectors);
    LoadVector(filenameBasisExpression, (float *) expressionBasisCPU, 4 * nVertices * NumberOfExpressions);

    auto shapeDevCPU = new float[NumberOfEigenvectors];
    auto expressionDevCPU = new float[NumberOfExpressions];
    LoadVector(filenameStdDevShape, shapeDevCPU, NumberOfEigenvectors);
    LoadVector(filenameStdDevExpression, expressionDevCPU, NumberOfExpressions);

	std::vector<float> alpha(NumberOfEigenvectors);
	for (int i = 0; i < NumberOfEigenvectors; i++) {
		alpha[i] = (i%2 == 0 ? 0.001 : -0.001);
	}

	std::vector<Eigen::Vector3f> interpolatedShape(nVertices);

	const Eigen::MatrixX3f averageShape = LoadOFF(filenameAverageMesh) / 1000000.0f;
	std::cout << "size: " << averageShape.rows() << "x" << averageShape.cols() << std::endl;
	for (int i = 0; i < nVertices; i++) {
		auto r = averageShape.row(i);
		interpolatedShape[i] << r(0), r(1), r(2);
	}

	std::cout << "Writing before ...";
	WriteOFF("out_Before.txt", interpolatedShape);
	std::cout << " K" << std::endl;

	// TODO: initialize interpolatedShape with vertices from averageMesh

	for (int i = 0; i < NumberOfEigenvectors; i++) {
		std::cout << "Processing vector " << i << " ..." << std::endl;
		for (int v = 0; v < nVertices; v++) {
			Eigen::Vector4f& basisVec = shapeBasisCPU[i*nVertices + v];
			interpolatedShape[i] += alpha[i] * basisVec.head<3>();
		}
	}

	std::cout << "Done! " << interpolatedShape[100] << std::endl;

	WriteOFF("out.txt", interpolatedShape);

	std::cin.get();
    return 0;
}
