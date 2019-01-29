#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

class SwitchControl {
public:
    SwitchControl(
            const std::vector<std::string>& states,
            const std::string& keyBackward,
            const std::string& keyForward,
            std::function<void(int, const std::vector<int>&)> callee);
    void addToVisualizer(pcl::visualization::PCLVisualizer& viewer, int viewportID);
    int getState();
private:
    static int s_currentY;
    static int s_currentID;
    static constexpr int c_textHeight = 12;

    const int m_currentY;
    const int m_currentID;
    const std::vector<std::string> m_states;
    const std::string m_keyBackward;
    const std::string m_keyForward;
    const std::function<void(int, const std::vector<int>&)> m_callee;

    int m_currentState;
    boost::signals2::connection m_callbackConnection;

    int m_selectedProp = 0;
    std::vector<int> m_props = std::vector<int>{0,0,0};
};
