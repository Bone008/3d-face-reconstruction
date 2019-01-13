#include <utility>

#include "Visualizer.h"
#include "SwitchControl.h"

int Visualizer::c_vpLeft = 1;
int Visualizer::c_vpRight = 2;
int Visualizer::c_vpBoth = 3;

Visualizer::Visualizer() :
m_viewer("Face Reconstruction"),
m_viewportSwitch(m_viewer, std::vector<std::string> {"Side by side", "Overlay"}, "", "v", [&](int state) { createViewports(); }) {

    m_viewer.setCameraPosition(-0.24917, -0.0187087, -1.29032, 0.0228136, -0.996651, 0.0785278);
    createViewports();
}

void Visualizer::run() {
    while (!m_viewer.wasStopped()) {
        m_viewer.spinOnce(500);
    }
}

pcl::visualization::PCLVisualizer &Visualizer::getViewer() {
    return m_viewer;
}

void Visualizer::setJohnPcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr johnPcl) {
    m_johnPcl = std::move(johnPcl);
    createViewports();
}

void Visualizer::setJohnFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr johnFeatures) {
    m_johnFeatures = std::move(johnFeatures);
    createViewports();
}


void Visualizer::setStevePcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr stevePcl) {
    m_stevePcl = std::move(stevePcl);
    createViewports();
}

void Visualizer::setSteveVertices(std::vector<pcl::Vertices> steveVertices) {
    m_steveVertices = std::move(steveVertices);
    createViewports();
}

void Visualizer::createViewports() {
    // clean existing viewports
    m_viewer.removeAllPointClouds(c_vpLeft);
    m_viewer.removeAllPointClouds(c_vpRight);
    m_viewer.removeAllPointClouds(c_vpBoth);

    // add new viewports
    int vpJohn;
    int vpSteve;
    if (m_viewportSwitch.getState() == 0) { // side by side
        m_viewer.createViewPort(0, 0, 0.5, 1, c_vpLeft);
        m_viewer.setBackgroundColor(0.2, 0, 0, c_vpLeft);
        m_viewer.createViewPort(0.5, 0, 1, 1, c_vpRight);
        m_viewer.setBackgroundColor(0, 0.2, 0, c_vpRight);
        vpJohn = c_vpLeft;
        vpSteve = c_vpRight;
    } else { // overlayed
        m_viewer.createViewPort(0, 0, 1, 1, c_vpBoth);
        m_viewer.setBackgroundColor(0, 0, 0.2, c_vpBoth);
        vpJohn = c_vpBoth;
        vpSteve = c_vpBoth;
    }

    // add john and steve
    if (m_johnPcl != 0) {
        m_viewer.addPointCloud<pcl::PointXYZRGB>(m_johnPcl, "john", vpJohn);
    }
    if (m_johnFeatures != 0) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> red(m_johnFeatures);
        m_viewer.addPointCloud<pcl::PointXYZRGB>(m_johnFeatures, red, "johnFeatures", vpJohn);
        m_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "johnFeatures");
    }
    if (m_stevePcl != 0 && !m_steveVertices.empty()) {
        m_viewer.addPolygonMesh<pcl::PointXYZRGB>(m_stevePcl, m_steveVertices, "steve", vpSteve);
    }

    // render once
    m_viewer.spinOnce();
}