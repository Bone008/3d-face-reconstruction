#include "stdafx.h"

pcl::visualization::PCLVisualizer viewer("PCL Viewer");

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

void highlightFeaturePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<Eigen::Vector3f> &featurePoints,
                            const std::string &name) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_highlight(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (auto const &point: featurePoints) {
        pcl::PointXYZRGB selected_point;
        selected_point.x = point[0];
        selected_point.y = point[1];
        selected_point.z = point[2];
        selected_point.r = 255;
        selected_point.g = 0;
        selected_point.b = 0;
        points_to_highlight->points.push_back(selected_point);
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> red(points_to_highlight);
    viewer.addPointCloud<pcl::PointXYZRGB>(points_to_highlight, red, name);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, name);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf &points) {
    const int nVertices = points.rows() / 3;
    pcl::PointXYZRGB tpl;
    tpl.r = tpl.g = tpl.b = 255;
    pcl::PointCloud<pcl::PointXYZRGB> cloud(nVertices, 1, tpl);
    for (int p = 0; p < nVertices; p++) {
        cloud.points[p].x = points(3 * p + 0);
        cloud.points[p].y = points(3 * p + 1);
        cloud.points[p].z = points(3 * p + 2);
    }
    return cloud.makeShared();
}

int main(int argc, char **argv) {
    // filenames
    std::string filenameBase = "../data/rgbd_face_dataset/";
    std::string filenamePcd = filenameBase + "006_00_cloud.pcd";
    std::string filenamePcdFeaturePoints = filenameBase + "006_00_features.points";

    std::string filenameBaseModel = "../data/MorphableModel/";
    std::string filenameAverageMesh = filenameBaseModel + "averageMesh.off";
    std::string filenameAverageMeshFeaturePoints = filenameBaseModel + "averageMesh_features.points";
    std::string filenameBasisShape = filenameBaseModel + "ShapeBasis.matrix";
    std::string filenameBasisExpression = filenameBaseModel + "ExpressionBasis.matrix";
    std::string filenameStdDevShape = filenameBaseModel + "StandardDeviationShape.vec";
    std::string filenameStdDevExpression = filenameBaseModel + "StandardDeviationExpression.vec";

    // load average shape
    const Eigen::VectorXf averageShape = LoadOFF(filenameAverageMesh) / 1000000.0f;

    // load average shape feature points
    auto averageShapeCloud = pointsToCloud(averageShape);
    FeaturePointExtractor averageFeatureExtractor(filenameAverageMeshFeaturePoints, averageShapeCloud);

    // load face model
    const unsigned int nVertices = 53490;
    const unsigned int nExpr = 76;
    const unsigned int nEigenVec = 160;

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

    auto expressionBasisRaw = new float[4 * nVertices * nExpr];
    LoadVector(filenameBasisExpression, expressionBasisRaw, 4 * nVertices * nExpr);

    // convert to matrix
    Eigen::Map<Eigen::MatrixXf> expressionBasisVec4(expressionBasisRaw, 4 * nVertices, nExpr);

    // discard 4th vertex coordinate
    Eigen::MatrixXf expressionBasis(3 * nVertices, nExpr);
    for (int i = 0; i < nVertices; i++) {
        expressionBasis.block(3 * i, 0, 3, nExpr) = expressionBasisVec4.block(4 * i, 0, 3, nExpr);
    }

    // not needed yet
    /*
    auto shapeDevRaw = new float[nEigenVec];
    LoadVector(filenameStdDevShape, shapeDevRaw, nEigenVec);

    auto expressionDevRaw = new float[nExpr];
    LoadVector(filenameStdDevExpression, expressionDevRaw, nExpr);
    */

    // load input point cloud
    Sensor sensor = VirtualSensor(filenamePcd);

    // load input feature points
    FeaturePointExtractor inputFeatureExtractor(filenamePcdFeaturePoints, sensor.m_cloud);

    // render input cloud
    viewer.addPointCloud<pcl::PointXYZRGB>(sensor.m_cloud, "inputCloud");
    highlightFeaturePoints(sensor.m_cloud, inputFeatureExtractor.m_points, "inputCloudFeatures");

    // transform average mesh using procrustes
    ProcrustesAligner pa;
    Eigen::Matrix4f pose = pa.estimatePose(averageFeatureExtractor.m_points, inputFeatureExtractor.m_points);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*averageShapeCloud, *transformedCloud, pose);

    // render transformed average mesh
    viewer.addPointCloud<pcl::PointXYZRGB>(transformedCloud, "inputTransformedCloud");
    highlightFeaturePoints(sensor.m_cloud, inputFeatureExtractor.m_points, "inputTransformedCloudFeatures");

    viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);
    viewer.spin();

    return 0;
}
