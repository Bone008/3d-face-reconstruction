#include <utility>

#include "Visualizer.h"
#include "SwitchControl.h"

/**
 * PCL Viewport ids
 */
int Visualizer::c_vpLeft = 1;
int Visualizer::c_vpRight = 2;
int Visualizer::c_vpBoth = 3;

Visualizer::Visualizer() :
        m_viewer("Face Reconstruction"),
        m_vpJohn(c_vpLeft),
        m_vpSteve(c_vpRight),
        m_changed(false) {

    m_viewer.setCameraPosition(
            -0.04, -0.005, 0.06, /* position */
            -0.1, -0.1, 1, /* view direction */
            0, -1, 0 /* view up */);

    createViewports();
    m_viewportSwitch = new SwitchControl(std::vector<std::string>{"Side by side", "Overlay"}, "", "v",
                     [&](int state) { std::cout << "callback2" << std::endl;m_changed = true; });
    addSwitch(m_viewportSwitch);
}

void Visualizer::run() {
    while (!m_viewer.wasStopped()) {
        runOnce();
    }
}

void Visualizer::runOnce() {
    {
        boost::lock_guard<boost::mutex> lock{m_mutex};
        if (m_changed) {
            createViewports();
            m_changed = false;
        }
    }
    m_viewer.spinOnce(500);
}

void Visualizer::setJohnPcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr johnPcl) {
    boost::lock_guard<boost::mutex> lock{m_mutex};

    m_johnPcl = std::move(johnPcl);
    m_changed = true;
}

void Visualizer::setJohnFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr johnFeatures) {
    boost::lock_guard<boost::mutex> lock{m_mutex};

    m_johnFeatures = std::move(johnFeatures);
    m_changed = true;
}


void Visualizer::setStevePcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr stevePcl) {
    boost::lock_guard<boost::mutex> lock{m_mutex};

    m_stevePcl = std::move(stevePcl);
    m_changed = true;
}

void Visualizer::setSteveVertices(std::vector<pcl::Vertices> steveVertices) {
    boost::lock_guard<boost::mutex> lock{m_mutex};

    m_steveVertices = std::move(steveVertices);
    m_changed = true;
}

void Visualizer::createViewports() {
    // clean existing viewports
    m_viewer.removeAllPointClouds(c_vpLeft);
    m_viewer.removeAllPointClouds(c_vpRight);
    m_viewer.removeAllPointClouds(c_vpBoth);
    m_viewer.removeAllShapes(c_vpLeft);
    m_viewer.removeAllShapes(c_vpRight);
    m_viewer.removeAllShapes(c_vpBoth);

    // add new viewports
    if (m_viewportSwitch == nullptr || m_viewportSwitch->getState() == 0) { // side by side
        m_viewer.createViewPort(0, 0, 0.5, 1, c_vpLeft);
        m_viewer.createViewPort(0.5, 0, 1, 1, c_vpRight);
        m_vpJohn = c_vpLeft;
        m_vpSteve = c_vpRight;
    } else { // overlayed
        m_viewer.createViewPort(0, 0, 1, 1, c_vpBoth);
        m_vpJohn = c_vpBoth;
        m_vpSteve = c_vpBoth;
    }

    // add switches
    for (auto& sw : m_switches) {
        sw->addToVisualizer(m_viewer, m_vpJohn);
    }

    // add john
    if (m_johnPcl != 0) {
        m_viewer.addPointCloud<pcl::PointXYZRGB>(m_johnPcl, "john", m_vpJohn);
    }

    // add johnFeatures
    if (m_johnFeatures != 0) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> red(m_johnFeatures);
        m_viewer.addPointCloud<pcl::PointXYZRGB>(m_johnFeatures, red, "johnFeatures", m_vpJohn);
        m_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "johnFeatures");
    }

    // add steve
    if (m_stevePcl != 0 && !m_steveVertices.empty()) {
        m_viewer.addPolygonMesh<pcl::PointXYZRGB>(m_stevePcl, m_steveVertices, "steve", m_vpSteve);
    }
}

void Visualizer::addSwitch(SwitchControl* newSwitch) {
    boost::lock_guard<boost::mutex> lock{m_mutex};

    m_switches.emplace_back(newSwitch);
    m_changed = true;
}
