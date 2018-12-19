#include "stdafx.h"
#include "CoarseAlignment.h"

Eigen::Matrix4f computeCoarseAlignment(const FaceModel& model, const Sensor& inputSensor)
{
	// transform average mesh using procrustes
	ProcrustesAligner pa;
	Matrix4f pose = pa.estimatePose(model.m_averageFeaturePoints, inputSensor.m_featurePoints);

	// TODO include peter's code for ICP here.

	return pose;
}
