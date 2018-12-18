#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <sstream>
#include "FeaturePointExtractor.h"
#include "ProcrustesAligner.h"
#include "Sensor.h"
#include "VirtualSensor.h"
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include "ceres/ceres.h"
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>


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

struct ExponentialResidual {
    // x is the source (average mesh), y is the target (input cloud)
    ExponentialResidual(float x, float y, int nEigenVec, Eigen::MatrixXf &base, int i)
            : x_(x), y_(y), nEigenVec_(nEigenVec), base_(base), i_(i) {}

    template <typename T>
    bool operator()(T const* alpha, T* residual) const {
        // solve 0 = target - (source + base * alpha)
        T baseTimesAlpha = T(0);
        for (int e = 0; e < nEigenVec_; e++) {
            baseTimesAlpha += T(base_(i_, e)) * alpha[e];
        }

        residual[0] = T(y_) - (T(x_) + baseTimesAlpha);
        return true;
    }

private:
    const float x_;
    const float y_;
    const int nEigenVec_;
    const Eigen::MatrixXf& base_;
    const int i_;
};

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud = sensor.m_cloud;

    // load input feature points
    FeaturePointExtractor inputFeatureExtractor(filenamePcdFeaturePoints, inputCloud);

    // transform average mesh using procrustes
    ProcrustesAligner pa;
    Eigen::Matrix4f pose = pa.estimatePose(inputFeatureExtractor.m_points, averageFeatureExtractor.m_points);
    pcl::PointCloud<pcl::PointXYZRGB> tmp;
    pcl::transformPointCloud(*inputCloud, tmp, pose);
    inputCloud = tmp.makeShared();

    // remove non-face points
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::getMinMax3D(*averageShapeCloud, min, max);

    Eigen::Vector4f size = max - min;
    min = min - size / 2;
    max = max + size / 2;
    min.w() = 1;
    max.w() = 1;

    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(min);
    boxFilter.setMax(max);
    boxFilter.setInputCloud(inputCloud);
    pcl::PointCloud<pcl::PointXYZRGB> out;
    boxFilter.filter(out);
    inputCloud = out.makeShared();

    // icp
    pcl::PointCloud<pcl::PointXYZRGB> out2;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(inputCloud);
    icp.setInputTarget(averageShapeCloud);
    icp.setMaxCorrespondenceDistance (0.01); // TODO tweak
    icp.setTransformationEpsilon (1e-8);// TODO tweak
    //icp.setEuclideanFitnessEpsilon (1); // TODO tweak
    icp.align(out2);
    inputCloud = out2.makeShared();

    // init alpha to 0
    double alpha[nEigenVec];
    for (int i = 0; i < nEigenVec; i++) {
        alpha[i] = 0;
    }

    // improve alpha 5 times
    for (int steps = 0; steps < 5; steps++) {

        // knn search using flann:
        // for every basel vertex, find closest vertex in input pcl
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(inputCloud);

        std::vector<int> pointID(1);
        std::vector<float> pointSqDist(1);

        std::vector<float> targetPositions;
        std::vector<float> sourcePositions;
        for (int i = 0; i < nVertices; i++) {
            kdtree.nearestKSearch(averageShapeCloud->points[i], 1, pointID, pointSqDist);
            pcl::PointXYZRGB t = inputCloud->points[pointID[0]];
            pcl::PointXYZRGB s = averageShapeCloud->points[i];

            // TODO check distance between t and s, maybe normal angles?
            if (pcl::squaredEuclideanDistance(t, s) > 0.01f)
                continue;

            targetPositions.push_back(t.x);
            targetPositions.push_back(t.y);
            targetPositions.push_back(t.z);
            sourcePositions.push_back(s.x);
            sourcePositions.push_back(s.y);
            sourcePositions.push_back(s.z);
        }

        std::cout << "knn search done (1)" << std::endl;

        // knn search using flann:
        // for every input pcl vertex, find closest basel vertex
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree2;
        kdtree2.setInputCloud(averageShapeCloud);

        for (int i = 0; i < inputCloud->width * inputCloud->height; i++) {
            kdtree2.nearestKSearch(inputCloud->points[i], 1, pointID, pointSqDist);
            pcl::PointXYZRGB t = averageShapeCloud->points[pointID[0]];
            pcl::PointXYZRGB s = inputCloud->points[i];

            // TODO check distance between t and s, maybe normal angles?
            if (pcl::squaredEuclideanDistance(t, s) > 0.01f)
                continue;

            targetPositions.push_back(t.x);
            targetPositions.push_back(t.y);
            targetPositions.push_back(t.z);
            sourcePositions.push_back(s.x);
            sourcePositions.push_back(s.y);
            sourcePositions.push_back(s.z);
        }

        std::cout << "knn search done (2)" << std::endl;

        // solve 0 = target - (source + base * alpha)
        ceres::Problem problem;

        for (int i = 0; i < 3 * nVertices; i++) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<ExponentialResidual, 1, nEigenVec>(
                            new ExponentialResidual(sourcePositions[i], targetPositions[i], nEigenVec, shapeBasis, i));
            problem.AddResidualBlock(cost_function, NULL, alpha);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << std::endl;

        Eigen::VectorXf alphaVec(nEigenVec);
        for (int i = 0; i < nEigenVec; i++) {
            alphaVec(i) = alpha[i];
        }
        const Eigen::VectorXf interpolatedShape(averageShape + shapeBasis * alphaVec);

        // modify point cloud
        averageShapeCloud = pointsToCloud(interpolatedShape);
    }

    // render transformed average mesh
    viewer.addPointCloud<pcl::PointXYZRGB>(averageShapeCloud, "inputTransformedCloud2");


    // render input cloud
    viewer.addPointCloud<pcl::PointXYZRGB>(inputCloud, "inputCloud");


    //highlightFeaturePoints(sensor.m_cloud, inputFeatureExtractor.m_points, "inputCloudFeatures");


    // render transformed average mesh
    //viewer.addPointCloud<pcl::PointXYZRGB>(transformedCloud, "inputTransformedCloud");
    //highlightFeaturePoints(sensor.m_cloud, inputFeatureExtractor.m_points, "inputTransformedCloudFeatures");


    viewer.setCameraPosition(0, 0, -1.5, 0, 0, 0);
    viewer.spin();

    return 0;
}
