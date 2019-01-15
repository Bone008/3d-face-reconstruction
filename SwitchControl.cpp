#include "SwitchControl.h"

int SwitchControl::s_currentY = 5;
int SwitchControl::s_currentID = 0;

SwitchControl::SwitchControl(pcl::visualization::PCLVisualizer& viewer, std::vector<std::string> states, std::string keyBackward, std::string keyForward,
                             std::function<void(int,const std::vector<int>&)> callee) : m_numStates((int) states.size()), m_currentState(0) {

    s_currentY += 2 * c_textHeight;

    int maxY = (m_numStates + 1) * c_textHeight + s_currentY;
    std::string idStr = "switch " + std::to_string(s_currentID++) + " ";
    std::string id2Str = "switch " + std::to_string(s_currentID++) + " ";
    viewer.addText("[ " + (keyBackward.empty() ? "" : (keyBackward + " | ")) + keyForward + " ]", 20, maxY,1, 0, 0, idStr + "keys");
    int i = 0;
    for (auto& state : states) {
        viewer.addText((i == 0 ? ">" : " ") + state, 30, maxY - (i + 1) * c_textHeight, 0, 1, 0, idStr + "option " + std::to_string(i));
        i++;
    }

    viewer.addText("[ Up | Down | Left | Right ]", 200, maxY,1, 0, 0, id2Str + "keys");
    const auto propertyNames = std::vector<std::string>{"age", "weight", "gender"};
    i = 0;
    for (auto& state : propertyNames) {
        viewer.addText((i == 0 ? ">" : " ") + state + ": " + std::to_string(m_props[i]), 200, maxY - (i + 1) * c_textHeight, 0, 1, 0, id2Str + "option " + std::to_string(i));
        i++;
    }

    viewer.registerKeyboardCallback([this, states, idStr, id2Str, maxY, propertyNames, keyBackward, keyForward, &viewer, callee](const pcl::visualization::KeyboardEvent keyEvent) {
        if (!keyEvent.keyUp())
            return;

        int dir = 0;
        if (keyEvent.getKeySym() == keyBackward)
            dir = -1;
        else if (keyEvent.getKeySym()== keyForward)
            dir = +1;

        viewer.updateText(" " + states[m_currentState], 30, maxY - (m_currentState + 1) * c_textHeight, 0, 1, 0, idStr + "option " + std::to_string(m_currentState));
        m_currentState = (m_currentState + m_numStates + dir) % m_numStates;
        viewer.updateText(">" + states[m_currentState], 30, maxY - (m_currentState + 1) * c_textHeight, 0, 1, 0, idStr + "option " + std::to_string(m_currentState));

        dir = 0;
        if (keyEvent.getKeySym() == "Up")
            dir = -1;
        else if (keyEvent.getKeySym()== "Down")
            dir = +1;

        int modDir = 0;
        if (keyEvent.getKeySym() == "Left")
            modDir = -1;
        else if (keyEvent.getKeySym()== "Right")
            modDir = +1;

        m_props[m_selectedProp] = m_props[m_selectedProp] + modDir;

        viewer.updateText(" " + propertyNames[m_selectedProp] + ": " + std::to_string(m_props[m_selectedProp]), 200, maxY - (m_selectedProp + 1) * c_textHeight, 0, 1, 0, id2Str + "option " + std::to_string(m_selectedProp));
        m_selectedProp = (m_selectedProp + propertyNames.size() + dir) % propertyNames.size();
        viewer.updateText(">" + propertyNames[m_selectedProp] + ": " + std::to_string(m_props[m_selectedProp]), 200, maxY - (m_selectedProp + 1) * c_textHeight, 0, 1, 0, id2Str + "option " + std::to_string(m_selectedProp));


        callee(m_currentState, m_props);
    });
    s_currentY += (m_numStates + 1) * c_textHeight;
}
