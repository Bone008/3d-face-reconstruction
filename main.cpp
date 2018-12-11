#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <sstream>
#include "FeaturePointExtractor.h"
#include "Sensor.h"
#include "VirtualSensor.h"

int activeCloud = 0;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
Sensor sensor;

const Eigen::VectorXf LoadOFF(std::string &filename) {
    std::ifstream in(filename, std::ifstream::in);
    if (!in) {
        std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
        throw;
    }

    std::string dummy;
    in >> dummy; // Header
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

void LoadVector(const std::string &filename, float *res, unsigned int length) {
    std::ifstream in(filename, std::ifstream::in | std::ifstream::binary);
    if (!in) {
        std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
        throw;
    }
    unsigned int numberOfEntries;
    in.read((char *) &numberOfEntries, sizeof(unsigned int));
    if (length == 0) length = numberOfEntries;
    in.read((char *) (res), length * sizeof(float));

    in.close();
}

// FIXME i am not used
float *LoadEigenvectors(const std::string &filename, unsigned int components, unsigned int numberOfEigenvectors) {
    auto *res = new float[components * numberOfEigenvectors];

    for (unsigned int i = 0; i < numberOfEigenvectors; i++) {
        std::stringstream ss;
        ss << filename << i << ".vec";

        LoadVector(ss.str().c_str(), &(res[components * i]), 0);
    }

    return res;
}

void onKeyboardEvent(const pcl::visualization::KeyboardEvent &event, void *) {
    if (event.getKeyCode() == 't' && event.keyDown()) {
        // show next point cloud
        activeCloud = (activeCloud + 1) % clouds.size();
        viewer.removeAllPointClouds(0);
        viewer.addPointCloud(clouds[activeCloud]);
        viewer.addPointCloud(sensor.m_cloud, "inputCloud");
    }
}

int main(int argc, char **argv) {
    // filenames
    std::string filenameBase = "../data/rgbd_face_dataset/"; // TODO maybe change
    std::string filenamePcd = filenameBase + "006_00_cloud.pcd";
    std::string filenameIndices = filenameBase + "006_00_features.points";

    // load point cloud
    sensor = VirtualSensor(filenamePcd);

    // load feature points
    //FeaturePointExtractor extractor(filenameIndices, sensor);

    // load face model
    const unsigned int nVertices = 53490;
    //const unsigned int nExpr = 76;
    const unsigned int nEigenVec = 160;

    std::string filenameBaseModel = "../data/MorphableModel/";
    std::string filenameAverageMesh = filenameBaseModel + "averageMesh.off";
    std::string filenameBasisShape = filenameBaseModel + "ShapeBasis.matrix";
    std::string filenameBasisExpression = filenameBaseModel + "ExpressionBasis.matrix";
    std::string filenameStdDevShape = filenameBaseModel + "StandardDeviationShape.vec";
    std::string filenameStdDevExpression = filenameBaseModel + "StandardDeviationExpression.vec";

    // load shape basis
    auto shapeBasisRaw = new float[4 * nVertices * nEigenVec];
    LoadVector(filenameBasisShape, shapeBasisRaw, 4 * nVertices * nEigenVec);

    // convert to matrix
    Eigen::Map<Eigen::MatrixXf> shapeBasisVec4(shapeBasisRaw, 4 * nVertices, nEigenVec);

    // discard 4th vertex coordinate
    Eigen::MatrixXf shapeBasis(3 * nVertices, nEigenVec);
    for (int i = 0; i < nVertices; i++) {
        shapeBasis.block(3 * i, 0, 3, nEigenVec) = shapeBasisVec4.block(4 * i, 0, 3, nEigenVec);
    }

    // not needed yet
    /*
    auto expressionBasisRaw = new float[4 * nVertices * nExpr];
    LoadVector(filenameBasisExpression, expressionBasisRaw, 4 * nVertices * nExpr);

    auto shapeDevRaw = new float[nEigenVec];
    LoadVector(filenameStdDevShape, shapeDevRaw, nEigenVec);

    auto expressionDevRaw = new float[nExpr];
    LoadVector(filenameStdDevExpression, expressionDevRaw, nExpr);
    */

    // load average shape
    const Eigen::VectorXf averageShape = LoadOFF(filenameAverageMesh) / 1000000.0f;

    // init normal distribution
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, 0.02f);

    for (int i = 0; i < 10; i++) {
        // generate alpha
        Eigen::VectorXf alpha(nEigenVec);
        for (int a = 0; a < nEigenVec; a++) {
            alpha(a) = distribution(generator);
        }

        // calculate face
        const Eigen::VectorXf interpolatedShape(averageShape + shapeBasis * alpha);

        // init point cloud
        pcl::PointXYZRGB tpl;
        tpl.r = tpl.g = tpl.b = 255;
        pcl::PointCloud<pcl::PointXYZRGB> cloud(nVertices, 1, tpl);
        for (int p = 0; p < nVertices; p++) {
            cloud.points[p].x = interpolatedShape(3 * p + 0);
            cloud.points[p].y = interpolatedShape(3 * p + 1);
            cloud.points[p].z = interpolatedShape(3 * p + 2);
        }

        // store point cloud
        auto cloudptr = cloud.makeShared();
        clouds.push_back(cloudptr);
    }

    viewer.setCameraPosition(0, 0, -0.5, 0, 1, 0);
    viewer.registerKeyboardCallback(onKeyboardEvent);
    viewer.addPointCloud<pcl::PointXYZRGB>(clouds[0]);

    while (not viewer.wasStopped()) {
        viewer.spinOnce(500);
    }

    return 0;
}
