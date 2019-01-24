#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include "SwitchControl.h"

class Visualizer {
public:
    Visualizer();

    void run();
    void runOnce();

    void setJohnPcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr johnPcl);
    void setJohnFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr johnFeatures);

    void setStevePcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr stevePcl);
    void setSteveVertices(std::vector<pcl::Vertices> steveVertices);

    void addSwitch(SwitchControl* newSwitch);

private:
    static int c_vpLeft;
    static int c_vpRight;
    static int c_vpBoth;

    int m_vpJohn;
    int m_vpSteve;

    bool m_changed;

    boost::mutex m_mutex;

    pcl::visualization::PCLVisualizer m_viewer;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_johnPcl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_johnFeatures;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_stevePcl;
    std::vector<pcl::Vertices> m_steveVertices;

    std::vector<SwitchControl*> m_switches;
    SwitchControl* m_viewportSwitch;

    void createViewports();

};
