#include "SwitchControl.h"

int SwitchControl::s_currentY = 5;
int SwitchControl::s_currentID = 0;

SwitchControl::SwitchControl(pcl::visualization::PCLVisualizer& viewer, std::vector<std::string> states, std::string keyBackward, std::string keyForward,
                             std::function<void(int)> callee) : m_numStates((int) states.size()), m_currentState(0) {

    s_currentY += 2 * c_textHeight;

    int maxY = (m_numStates + 1) * c_textHeight + s_currentY;
    std::string idStr = "switch " + std::to_string(s_currentID++) + " ";
    viewer.addText("[ " + (keyBackward.empty() ? "" : (keyBackward + " | ")) + keyForward + " ]", 20, maxY,1, 0, 0, idStr + "keys");
    int i = 0;
    for (auto& state : states) {
        viewer.addText((i == 0 ? ">" : " ") + state, 30, maxY - (i + 1) * c_textHeight, 0, 1, 0, idStr + "option " + std::to_string(i));
        i++;
    }
    viewer.registerKeyboardCallback([this, states, idStr, maxY, keyBackward, keyForward, &viewer, callee](const pcl::visualization::KeyboardEvent keyEvent) {
        if (!keyEvent.keyUp())
            return;

        int dir = 0;
        if (keyEvent.getKeySym() == keyBackward)
            dir = -1;
        else if (keyEvent.getKeySym()== keyForward)
            dir = +1;
        else
            return;

        viewer.updateText(" " + states[m_currentState], 30, maxY - (m_currentState + 1) * c_textHeight, 0, 1, 0, idStr + "option " + std::to_string(m_currentState));

        m_currentState = (m_currentState + m_numStates + dir) % m_numStates;

        viewer.updateText(">" + states[m_currentState], 30, maxY - (m_currentState + 1) * c_textHeight, 0, 1, 0, idStr + "option " + std::to_string(m_currentState));

        callee(m_currentState);
    });
    s_currentY += (m_numStates + 1) * c_textHeight;
}

int SwitchControl::getState() {
    return m_currentState;
}
