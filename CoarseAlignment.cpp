#include "stdafx.h"
#include "CoarseAlignment.h"
#include <pcl/registration/icp.h>
#include "utils.h"
#include "FaceModel.h"
#include "Sensor.h"
#include "ProcrustesAligner.h"

Eigen::Matrix4f computeCoarseAlignment(const FaceModel& model, const Sensor& inputSensor)
{
	// transform average mesh using procrustes
	std::cout << "  procrustes ..." << std::endl;
	ProcrustesAligner pa;
	Eigen::Matrix4f pose = pa.estimatePose(model.m_averageFeaturePoints, inputSensor.m_featurePoints);

	// transform average mesh using icp
	std::cout << "  icp ... " << std::flush;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr procrustesModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::transformPointCloud(*pointsToCloud(model.m_averageShapeMesh.vertices), *procrustesModelCloud, pose);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(procrustesModelCloud);
	icp.setInputTarget(inputSensor.m_cloud);
	icp.setMaxCorrespondenceDistance (0.2); // TODO tweak
	icp.setTransformationEpsilon (1e-8);// TODO tweak
	icp.setEuclideanFitnessEpsilon (0.001); // TODO tweak

    pcl::PointCloud<pcl::PointXYZRGB> icpAlignedCloud;
    icp.align(icpAlignedCloud);

	if (icp.hasConverged()) {
		std::cout << "converged" << std::endl;
		pose = icp.getFinalTransformation() * pose;
	} else {
		std::cout << "failed" << std::endl;
	}

	return pose;
}
