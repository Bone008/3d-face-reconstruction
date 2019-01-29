#include "stdafx.h"
#include "Rasterizer.h"

using namespace Eigen;

// Helper struct for computation of barycentric coordinates.
struct BarycentricTransform {
private:
	Vector2f offset;
	Matrix2f Ti;
public:
	BarycentricTransform(const Vector2f& s0, const Vector2f& s1, const Vector2f& s2) :
		offset(s2) {
		Matrix2f T;
		T << (s0 - s2), (s1 - s2);
		Ti = T.inverse();
	}
	Vector3f operator()(const Vector2f& v) const {
		Vector2f b;
		b = Ti * (v - offset);
		return Vector3f(b[0], b[1], 1.0f - b[0] - b[1]);
	}
};


void Rasterizer::compute(const FaceParameters& params) {
	std::cout << "          Alpha: " << params.alpha.head<4>().transpose() << std::endl;
	std::cout << "          Beta: " << params.beta.head<4>().transpose() << ", etc." << std::endl;
	std::cout << "          Rasterization: project ..." << std::flush;

	Matrix3Xf projectedVertices;
	Matrix4Xi vertexAlbedos;
	Matrix3Xf worldNormals;
	project(params, projectedVertices, vertexAlbedos, worldNormals);
	rasterize(projectedVertices, vertexAlbedos, worldNormals, params.gamma);

	numCalls++;
}

void Rasterizer::project(const FaceParameters& params, Matrix3Xf& outProjectedVertices, Matrix4Xi& outVertexAlbedos, Matrix3Xf& outWorldNormals) {
	VectorXf flatVertices = model.computeShape(params);
	Matrix3Xf worldVertices = pose.topLeftCorner<3, 3>() * Map<Matrix3Xf>(flatVertices.data(), 3, model.getNumVertices());
	worldVertices.colwise() += pose.topRightCorner<3, 1>();
	// Project to screen space.
	outProjectedVertices = intrinsics * worldVertices;

	outVertexAlbedos = model.computeColors(params);

	Matrix3Xf normals = model.computeNormals(flatVertices);
	// Only apply rotation of pose to normals.
	outWorldNormals = pose.topLeftCorner<3, 3>() * normals;
}

void Rasterizer::rasterize(const Matrix3Xf& projectedVertices, const Matrix4Xi& vertexAlbedos, const Eigen::Matrix3Xf& worldNormals, const Eigen::VectorXf& gamma) {
	// Reset output.
	std::fill(pixelResults.begin(), pixelResults.end(), PixelData());

	std::cout << " rasterize ..." << std::flush;

	ArrayXXf depthBuffer(frameSize.x(), frameSize.y());
	depthBuffer.setConstant(std::numeric_limits<float>::infinity());

	Vector3f L = Vector3f(0, 0, -1);

    float c1 = 0.429043f;
    float c2 = 0.511664f;
    float c3 = 0.743125f;
    float c4 = 0.886227f;
    float c5 = 0.247708f;

    float _L00 = gamma(0);
    float _L1N1 = gamma(1);
    float _L10 = gamma(2);
    float _L11 = gamma(3);
    float _L2N2 = gamma(4);
    float _L2N1 = gamma(5);
    float _L20 = gamma(6);
    float _L21 = gamma(7);
    float _L22 = gamma(8);

	const Matrix3Xi& triangles = model.m_averageMesh.triangles;

	for (size_t t = 0; t < triangles.cols(); t++) {
		const auto& indices = triangles.col(t);
		Vector3f v0 = projectedVertices.col(indices(0));
		Vector3f v1 = projectedVertices.col(indices(1));
		Vector3f v2 = projectedVertices.col(indices(2));

		Vector3f n0 = worldNormals.col(indices(0));
		Vector3f n1 = worldNormals.col(indices(1));
		Vector3f n2 = worldNormals.col(indices(2));

		// Get vertices in pixel space.
		auto s0 = ((v0.head<2>() / v0.z()).array()).matrix();
		auto s1 = ((v1.head<2>() / v1.z()).array()).matrix();
		auto s2 = ((v2.head<2>() / v2.z()).array()).matrix();

		// Calculate bounds of triangle on screen.
		Array2i boundsMinPx = s0.array().min(s1.array()).min(s2.array()).cast<int>();
		Array2i boundsMaxPx = s0.array().max(s1.array()).max(s2.array()).cast<int>();
		//Array2i boundsMinPx = ((0.5f*boundsMin + 0.5f) * frameSize.cast<float>()).cast<int>();
		//Array2i boundsMaxPx = ((0.5f*boundsMax + 0.5f) * frameSize.cast<float>()).cast<int>();
		boundsMaxPx += 1;

		// Clip to actual frame buffer region.
		boundsMinPx = boundsMinPx.max(Array2i(0, 0));
		boundsMaxPx = boundsMaxPx.min(frameSize);

		BarycentricTransform bary(s0, s1, s2);

		for (int y = boundsMinPx.y(); y < boundsMaxPx.y(); y++) {
			for (int x = boundsMinPx.x(); x < boundsMaxPx.x(); x++) {
				/*Array2f pos(x, y);
				pos /= frameSize.cast<float>();
				pos = (pos - 0.5f) * 2.0f;

				Vector3f baryCoords = bary(pos.matrix());*/
				Vector2f pixelCenter(x + 0.5f, y + 0.5f);
				Vector3f baryCoords = bary(pixelCenter);

				if ((baryCoords.array() <= 1.0f).all() && (baryCoords.array() >= 0.0f).all()) {
					float depth = baryCoords.dot(Vector3f(v0.z(), v1.z(), v2.z()));
					if (depth < depthBuffer(x, y)) {
						depthBuffer(x, y) = depth;
						PixelData& out = pixelResults[y * frameSize.x() + x];
						out.isValid = true;
						out.pixelCenter = pixelCenter;
						out.vertexIndices[0] = indices(0);
						out.vertexIndices[1] = indices(1);
						out.vertexIndices[2] = indices(2);
						out.barycentricCoordinates = baryCoords;

						Vector3f n = baryCoords(0) * n0 + baryCoords(1) * n1 + baryCoords(2) * n2;
						n.normalize();

						out.normal = n;

						float E = c1 * _L22 * (n(0)*n(0) - n(1)*n(1)) +
                                  c3 * _L20 * (n(2) * n(2)) +
                                  c4 * _L00 -
                                  c5 * _L20 +
                                  2.0 * c1 * (_L2N2 * n(0) * n(1) + _L21 * n(0) * n(2) + _L2N1 * n(1) * n(2)) +
                                  2.0 * c2 * (_L11 * n(0) + _L1N1 * n(1) + _L10 * n(2));

						Vector3f albedo =
							baryCoords(0) * vertexAlbedos.col(indices(0)).head<3>().cast<float>() +
							baryCoords(1) * vertexAlbedos.col(indices(1)).head<3>().cast<float>() +
							baryCoords(2) * vertexAlbedos.col(indices(2)).head<3>().cast<float>();

						//out.albedo = (-n + Vector3f(1.0f, 1.0f, 1.0f)) * 0.5f * 255.0f;// albedo * E;
						out.albedo = albedo * E;

						//out.albedo = Vector3f(E, E, E);

					}
				}
			}
		}
	}

	size_t filledPx = std::count_if(pixelResults.begin(), pixelResults.end(), [](const PixelData& px) { return px.isValid; });
	std::cout << " (valid pixels: " << filledPx << ")";
	writeDebugImages();
	std::cout << " done!" << std::endl;
}


Vector3f Rasterizer::getAverageColor() {
	size_t num = 0;
	Vector3f colorSum;
	colorSum.setZero();
	for (const PixelData& pixel : pixelResults) {
		if (pixel.isValid) {
			num++;
			colorSum += pixel.albedo;
		}
	}
	return colorSum / num;
}


void Rasterizer::writeDebugImages() {
	std::cout << " saving bmp ..." << std::flush;
	BMP bmp(frameSize.x(), frameSize.y());
	BMP bmpCol(frameSize.x(), frameSize.y());
	// replace infinity values in buffer with 0
	for (size_t i = 0; i < depthBuffer.size(); i++) {
		if (std::isinf(depthBuffer.data()[i]))
			depthBuffer.data()[i] = 0;
	}
	// TODO make scale respect minCoeff() as well for better color range
	const float scale = depthBuffer.maxCoeff();
	for (int i = 0; i < depthBuffer.size(); i++) {
		Array4i depthCol;
		if (depthBuffer.data()[i] == 0) {
			depthCol = Array4i(0, 0, 30, 255);
		}
		else {
			int c = int(depthBuffer.data()[i] / scale * 255);
			depthCol = Array4i(c, c, c, 255);
		}
		bmp.data[4 * i + 2] = depthCol[0];
		bmp.data[4 * i + 1] = depthCol[1];
		bmp.data[4 * i + 0] = depthCol[2];
		bmp.data[4 * i + 3] = depthCol[3];

		Array4i col(0, 0, 0, 255);
		auto& result = pixelResults[i];
		if (result.isValid) {
			col.head<3>() = result.albedo.cast<int>();
		}

		bmpCol.data[4 * i + 2] = col[0];
		bmpCol.data[4 * i + 1] = col[1];
		bmpCol.data[4 * i + 0] = col[2];
		bmpCol.data[4 * i + 3] = col[3];
	}

	char filename[100];
	sprintf(filename, "depthmap_%d.bmp", numCalls);
	bmp.write(filename);
	sprintf(filename, "ecolmap_%d.bmp", numCalls);
	bmpCol.write(filename);
}
