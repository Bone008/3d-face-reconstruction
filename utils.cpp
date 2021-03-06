#include "stdafx.h"
#include "utils.h"
#include "BMP.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf& points) {
    const unsigned int nVertices = points.rows() / 3;
    pcl::PointXYZRGB tpl;
    tpl.r = tpl.g = tpl.b = 255;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(nVertices, 1, tpl));
    for (int p = 0; p < nVertices; p++) {
        cloud->points[p].x = points(3 * p + 0);
        cloud->points[p].y = points(3 * p + 1);
        cloud->points[p].z = points(3 * p + 2);
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf& points, const Eigen::Matrix4Xi& vertexColors) {
    const unsigned int nVertices = points.rows() / 3;
    pcl::PointXYZRGB tpl;
    tpl.r = tpl.g = tpl.b = 255;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(nVertices, 1, tpl));
    for (int p = 0; p < nVertices; p++) {
        cloud->points[p].x = points(3 * p + 0);
        cloud->points[p].y = points(3 * p + 1);
        cloud->points[p].z = points(3 * p + 2);
        cloud->points[p].r = vertexColors(0, p);
        cloud->points[p].g = vertexColors(1, p);
        cloud->points[p].b = vertexColors(2, p);
        // TODO change type everywhere to pcl::PointXYZRGBA
        //cloud->points[p].a = vertexColors(0, p);
    }
    return cloud;
}

// Version with normals.
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointsToCloud(const Eigen::VectorXf& points, const Eigen::Matrix3Xf& normals) {
	const unsigned int nVertices = points.rows() / 3;
	pcl::PointXYZRGBNormal tpl;
	tpl.r = tpl.g = tpl.b = 255;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>(nVertices, 1, tpl));
	for (int p = 0; p < nVertices; p++) {
		cloud->points[p].x = points(3 * p + 0);
		cloud->points[p].y = points(3 * p + 1);
		cloud->points[p].z = points(3 * p + 2);
		cloud->points[p].normal_x = normals(0, p);
		cloud->points[p].normal_y = normals(1, p);
		cloud->points[p].normal_z = normals(2, p);
	}
	return cloud;
}

std::vector<pcl::Vertices> trianglesToVertexList(const Eigen::Matrix3Xi& triangles) {
    std::vector<pcl::Vertices> vertices(triangles.cols());

    for (int c = 0; c < triangles.cols(); c++) {
        pcl::Vertices v;
        for (int r = 0; r < 3; r++) {
            v.vertices.push_back(triangles(r, c));
        }
        vertices.push_back(v);
    }

    return vertices;
}


// Saves a BMP image using the colors provided by a callback (x, y) --> RGB.
void saveBitmap(const char* filename, int width, int height, std::function<Eigen::Vector3i(unsigned int x, unsigned int y)> pixelColorProvider) {
	BMP bmp(width, height);
	for (unsigned int y = 0; y < height; y++) {
		for (unsigned int x = 0; x < width; x++) {
			Eigen::Vector3i color = pixelColorProvider(x, y).cwiseMax(0).cwiseMin(255);
			// BMP stores data in BGRA order, the provider returns RGBA.
			bmp.data[4 * (y * width + x) + 2] = color(0);
			bmp.data[4 * (y * width + x) + 1] = color(1);
			bmp.data[4 * (y * width + x) + 0] = color(2);
			bmp.data[4 * (y * width + x) + 3] = 255;
		}
	}
	bmp.write(filename);
}

// Saves a BMP image using the colors provided by a callback (x, y) --> RGBA.
void saveBitmapAlpha(const char* filename, int width, int height, std::function<Eigen::Vector4i(unsigned int x, unsigned int y)> pixelColorProvider) {
	BMP bmp(width, height);
	for (unsigned int y = 0; y < height; y++) {
		for (unsigned int x = 0; x < width; x++) {
			Eigen::Vector4i color = pixelColorProvider(x, y).cwiseMax(0).cwiseMin(255);
			// BMP stores data in BGRA order, the provider returns RGBA.
			bmp.data[4 * (y * width + x) + 2] = color(0);
			bmp.data[4 * (y * width + x) + 1] = color(1);
			bmp.data[4 * (y * width + x) + 0] = color(2);
			bmp.data[4 * (y * width + x) + 3] = color(3);
		}
	}
	bmp.write(filename);
}
