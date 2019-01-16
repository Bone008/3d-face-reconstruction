#pragma once
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <sstream>
#include "cxxopts.hpp"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <ceres/ceres.h>
