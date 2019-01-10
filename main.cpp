#include "stdafx.h"
#include "VirtualSensor.h"
#include "OpenNI2Sensor.h"
#include "FaceModel.h"
#include "CoarseAlignment.h"
#include "Optimizer.h"
#include "utils.h"

const std::string baseModelDir = "../data/MorphableModel/";
const std::string inputFaceBaseDir = "../data/rgbd_face_dataset/";
const std::string inputFacePcdFile = inputFaceBaseDir + "006_00_cloud.pcd";
const std::string inputFeaturePointsFile = inputFaceBaseDir + "006_00_features.points";
const std::string sensorOutputFacePcdFile = inputFaceBaseDir + "temp_cloud.pcd";
const std::string sensorFeaturePointsFile = inputFaceBaseDir + "temp_features.points";
const bool useDepthSensor = true;

pcl::visualization::PCLVisualizer viewer("PCL Viewer");

void highlightFeaturePoints(std::vector<Eigen::Vector3f> featurePoints,
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


int main(int argc, char **argv) {
	std::cout << "Loading face model ..." << std::endl;
	FaceModel model(baseModelDir);

    // load cloud from sensor/file
    std::cout << "Loading point cloud ..." << std::endl;
    const Sensor* inputSensorPtr;
    if (useDepthSensor) {
        inputSensorPtr = new OpenNI2Sensor(sensorOutputFacePcdFile, sensorFeaturePointsFile);
    } else {
        inputSensorPtr = new VirtualSensor(inputFacePcdFile, inputFeaturePointsFile);
    }
    const Sensor& inputSensor = *inputSensorPtr; // yes, i really don't want to use pointers

    // visualize input point cloud (John)
	viewer.addPointCloud<pcl::PointXYZRGB>(inputSensor.m_cloud, "inputCloud");
	highlightFeaturePoints(inputSensor.m_featurePoints, "inputCloudFeatures");

    std::cout << "Coarse alignment ..." << std::endl;
    Eigen::Matrix4f pose = computeCoarseAlignment(model, inputSensor);

    std::cout << "Optimizing parameters ..." << std::endl;
    FaceParameters params = optimizeParameters(model, pose, inputSensor);
    Eigen::VectorXf finalShape = model.computeShape(params);

    std::cout << "Done!" << std::endl;

    // visualize final reconstruction (Steve)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*pointsToCloud(finalShape, model.m_averageShapeMesh.vertexColors), *transformedCloud, pose);

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr transformedMeshCloud(transformedCloud);
    viewer.addPolygonMesh<pcl::PointXYZRGB>(transformedMeshCloud, trianglesToVertexList(model.m_averageShapeMesh.triangles));

	viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);
	while (!viewer.wasStopped()) {
		// TODO: react to input to modify params and call viewer.updatePointCloud(...)
		viewer.spinOnce(500);
	}
	return 0;
}
