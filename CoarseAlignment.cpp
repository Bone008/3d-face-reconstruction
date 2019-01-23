#include "stdafx.h"
#include "CoarseAlignment.h"
#include <pcl/registration/icp.h>
#include "utils.h"
#include "FaceModel.h"
#include "Sensor.h"
#include "ProcrustesAligner.h"

using namespace Eigen;

Matrix4f computeCoarseAlignmentProcrustes(const FaceModel& model, const Sensor& inputSensor) {
	std::cout << "  procrustes ..." << std::endl;
	ProcrustesAligner pa;
	return pa.estimatePose(model.m_averageFeaturePoints, inputSensor.m_featurePoints);
}

Eigen::Matrix4f computeCoarseAlignmentICP(const FaceModel& model, const Sensor& inputSensor, const Eigen::Matrix4f& initialPose) {
	std::cout << "  icp ... " << std::flush;
	Matrix3Xf modelNormals = model.computeNormals(model.m_averageMesh.vertices);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelCloud = pointsToCloud(model.m_averageMesh.vertices, modelNormals);

	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	icp.setInputSource(modelCloud);
	icp.setInputTarget(inputSensor.compute_normals());

	// TODO set params dependent on input cloud features' scale (e.g. dependent on distance between eyes)
	icp.setMaxCorrespondenceDistance (0.02); // TODO tweak
	icp.setTransformationEpsilon(1e-5);// TODO tweak
	icp.setEuclideanFitnessEpsilon (1e-4); // TODO tweak

    pcl::PointCloud<pcl::PointXYZRGBNormal> icpAlignedCloud;
    icp.align(icpAlignedCloud, initialPose);

	if (icp.hasConverged()) {
		std::cout << "converged" << std::endl;
		return icp.getFinalTransformation();
	} else {
		std::cout << "failed" << std::endl;
		return Matrix4f::Identity();
	}
}
