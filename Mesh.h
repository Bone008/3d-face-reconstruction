#pragma once

// Generic representation of a mesh composed of triangles and colored vertices.
struct Mesh {
	Eigen::VectorXf vertices;
	Eigen::Matrix4Xi vertexColors;
	Eigen::Matrix3Xi triangles;
};
