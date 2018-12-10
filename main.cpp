#include <Eigen/Eigen>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "FeaturePointExtractor.h"
#include "Sensor.h"
#include "VirtualSensor.h"

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
    // filenames
    std::string filenameBase = "../data/rgbd_face_dataset/"; // TODO maybe change
    std::string filenamePcd = filenameBase + "006_00_cloud.pcd";
    std::string filenameIndices = filenameBase + "006_00_features.points";

    // load point cloud
    Sensor sensor = VirtualSensor(filenamePcd);

    // load feature points
    FeaturePointExtractor extractor(filenameIndices, sensor);

    // load face model
    int nVertices = 53498;
    int NumberOfExpressions = 76;
    int NumberOfEigenvectors = 160;

    std::string filenameBaseModel = "../../";
    std::string filenameBasisShape = filenameBaseModel + "ShapeBasis.matrix";
    std::string filenameBasisExpression = filenameBaseModel + "ExpressionBasis.matrix";
    std::string filenameStdDevShape = filenameBaseModel + "StandardDeviationShape.vec";
    std::string filenameStdDevExpression = filenameBaseModel + "StandardDeviationExpression.vec";

    // the next two floats were originally float4
    auto shapeBasisCPU = new float[4 * nVertices * NumberOfEigenvectors];
    auto expressionBasisCPU = new float[4 * nVertices * NumberOfExpressions];
    LoadVector(filenameBasisShape, (float *) shapeBasisCPU, 4 * nVertices * NumberOfEigenvectors);
    LoadVector(filenameBasisExpression, (float *) expressionBasisCPU, 4 * nVertices * NumberOfExpressions);

    auto shapeDevCPU = new float[NumberOfEigenvectors];
    auto expressionDevCPU = new float[NumberOfExpressions];
    LoadVector(filenameStdDevShape, shapeDevCPU, NumberOfEigenvectors);
    LoadVector(filenameStdDevExpression, expressionDevCPU, NumberOfExpressions);

    for (int i = 0; i < 24; i++) {
        std::cout << expressionDevCPU[i] << std::endl;
    }

    return 0;
}
