#pragma once
#include "FaceModel.h"
class Sensor;

FaceParameters optimizeParameters(FaceModel& model, const Eigen::Matrix4f& pose, const Sensor& inputSensor,
        std::function<void(const FaceParameters& params)> intermediateResultCallback);
