#pragma once
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

class Sensor {
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	std::vector<Eigen::Vector3f>  m_featurePoints;
	Eigen::Matrix3f m_cameraIntrinsics;

    explicit Sensor() : m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {
		
	}

	pcl::PointCloud<pcl::Normal>::Ptr compute_normals() {
		// load point cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud = m_cloud;
		// estimate normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		ne.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::COVARIANCE_MATRIX);
		ne.setNormalSmoothingSize(10.0f);
		ne.setDepthDependentSmoothing(true);
		ne.compute(*normals);

		// visualize normals
		//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals);
		
		return normals;
	}

};
