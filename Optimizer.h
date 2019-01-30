#pragma once
#include "FaceModel.h"
#include "Rasterizer.h"
class Sensor;

struct OptimizerOutput {
	std::shared_ptr<Rasterizer> rasterizer;
	Eigen::Vector3f colorDelta;
};

FaceParameters optimizeParameters(FaceModel& model, const Eigen::Matrix4f& pose, const Sensor& inputSensor,
		OptimizerOutput& outputInfo,
        std::function<void(const FaceParameters& params)> intermediateResultCallback);

void saveCompositeImage(const char* filename, const pcl::PointCloud<pcl::PointXYZRGB>& inputCloud, OptimizerOutput& state, const FaceParameters& params);
