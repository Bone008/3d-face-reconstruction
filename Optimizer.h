#pragma once
#include "FaceModel.h"
#include "Sensor.h"

FaceParameters optimizeParameters(FaceModel& model, const Eigen::Matrix4f& pose, const Sensor& inputSensor);
