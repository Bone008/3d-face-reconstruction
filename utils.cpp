#include <pcl/common/common.h>
#include "utils.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToCloud(const Eigen::VectorXf &points) {
    const int nVertices = points.rows() / 3;
    pcl::PointXYZRGB tpl;
    tpl.r = tpl.g = tpl.b = 255;
    pcl::PointCloud<pcl::PointXYZRGB> cloud(nVertices, 1, tpl);
    for (int p = 0; p < nVertices; p++) {
        cloud.points[p].x = points(3 * p + 0);
        cloud.points[p].y = points(3 * p + 1);
        cloud.points[p].z = points(3 * p + 2);
    }
    return cloud.makeShared();
}
