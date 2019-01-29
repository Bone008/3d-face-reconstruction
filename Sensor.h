#pragma once
#include <pcl/features/integral_image_normal.h>
#include "FeaturePointExtractor.h"

class Sensor {
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	std::vector<Eigen::Vector3f>  m_featurePoints;
	Eigen::Matrix3f m_cameraIntrinsics;

    explicit Sensor() : m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {
		
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr compute_normals() const {
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

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dst(new pcl::PointCloud<pcl::PointXYZRGBNormal>); // To be created
		// Initialization part
		dst->width = cloud->width;
		dst->height = cloud->height;
		dst->is_dense = true;
		dst->points.resize(dst->width * dst->height);

		// Assignment part
		for (int i = 0; i < normals->points.size(); i++)
		{
			dst->points.at(i).x = cloud->points.at(i).x;
			dst->points.at(i).y = cloud->points.at(i).y;
			dst->points.at(i).z = cloud->points.at(i).z;

			dst->points.at(i).r = cloud->points.at(i).r;
			dst->points.at(i).g = cloud->points.at(i).g;
			dst->points.at(i).b = cloud->points.at(i).b;

			// cloud_normals -> Which you have already have; generated using pcl example code 

			dst->points.at(i).curvature = normals->points[i].curvature;

			dst->points.at(i).normal_x = normals->points[i].normal_x;
			dst->points.at(i).normal_y = normals->points[i].normal_y;
			dst->points.at(i).normal_z = normals->points[i].normal_z;
		}
		// visualize normals
		//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals);
		return dst;
	}

};
