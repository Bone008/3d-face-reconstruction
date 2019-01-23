#pragma once

class FaceModel;
class Sensor;

// returns pose
Eigen::Matrix4f computeCoarseAlignmentProcrustes(const FaceModel& model, const Sensor& inputSensor);
Eigen::Matrix4f computeCoarseAlignmentICP(const FaceModel& model, const Sensor& inputSensor, const Eigen::Matrix4f& initialPose);