#pragma once

class FaceModel;
class Sensor;

// returns pose
Eigen::Matrix4f computeCoarseAlignment(const FaceModel& model, const Sensor& inputSensor);