#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

class SwitchControl {
public:
    SwitchControl(pcl::visualization::PCLVisualizer& viewer, std::vector<std::string> states, std::string keyBackward, std::string keyForward, std::function<void(int)> callee);

private:
    static int s_currentY;
    static int s_currentID;
    static constexpr int c_textHeight = 12;

    int m_numStates;
    int m_currentState;
};
