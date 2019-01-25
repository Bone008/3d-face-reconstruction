#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

class SwitchControl {
public:
    SwitchControl(pcl::visualization::PCLVisualizer& viewer, std::vector<std::string> states, std::string keyBackward, std::string keyForward, int place, std::function<void(int,const std::vector<int>&)> callee);

private:
    static int s_currentY;
    static int s_currentID;
    static constexpr int c_textHeight = 12;

    int m_numStates;
    int m_currentState;

    int m_selectedProp = 0;
    std::vector<int> m_props = std::vector<int>{0,0,0};
};
