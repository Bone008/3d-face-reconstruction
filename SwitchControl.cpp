#include <utility>
// TODO utility needed?
#include "SwitchControl.h"

int SwitchControl::s_currentY = 5;
int SwitchControl::s_currentID = 0;

SwitchControl::SwitchControl(
        const std::vector<std::string>& states,
        const std::string& keyBackward,
        const std::string& keyForward,
        std::function<void(int, const std::vector<int>&)> callee)
        :
        m_currentID(s_currentID++),
        m_currentY(s_currentY + 2 * c_textHeight),
        m_states(states),
        m_keyBackward(keyBackward),
        m_keyForward(keyForward),
        m_currentState(0),
        m_callee(std::move(callee)) {

    s_currentY += 2 * c_textHeight + (states.size() + 1) * c_textHeight;
}

void SwitchControl::addToVisualizer(pcl::visualization::PCLVisualizer &viewer, int viewportID) {
    int numStates = (int) m_states.size();
    int maxY = (numStates + 1) * c_textHeight + m_currentY;
    std::string idStr = "switch " + std::to_string(m_currentID) + " ";
    std::string id2Str = "switch2 " + std::to_string(m_currentID) + " ";

    std::vector<std::string> propertyNames;
    if (m_keyForward == "Right") {
        maxY = 65;
        // add 4-switch keys
        viewer.addText(
                "[ Up | Down | Left | Right ]",
                200, maxY,
                1, 0, 0,
                id2Str + "keys",
                viewportID);
        propertyNames = std::vector<std::string>{"age", "weight", "gender"};
        int i = 0;
        for (auto& state : propertyNames) {
            viewer.addText(
                    (i == m_selectedProp ? ">" : " ") + state + ": " + std::to_string(m_props[i]),
                    200, maxY - (i + 1) * c_textHeight,
                    0, 1, 0,
                    id2Str + "option " + std::to_string(i),
                    viewportID);
            i++;
        }
    } else {
        // add header text
        viewer.addText(
                "[ " + (m_keyBackward.empty() ? "" : (m_keyBackward + " | ")) + m_keyForward + " ]",
                20, maxY,
                1, 0, 0,
                idStr + "keys",
                viewportID);

        // add option texts
        int i = 0;
        for (auto &state : m_states) {
            viewer.addText(
                    (i == m_currentState ? ">" : " ") + state,
                    30, maxY - (i + 1) * c_textHeight,
                    0, 1, 0,
                    idStr + "option " + std::to_string(i),
                    viewportID);
            i++;
        }
    }

    // register keypressed callback
    if (!m_callbackConnection.connected()) {
        m_callbackConnection = viewer.registerKeyboardCallback(
                [this, idStr, maxY, numStates, propertyNames, id2Str, &viewer](const pcl::visualization::KeyboardEvent keyEvent) {
                    if (!keyEvent.keyUp())
                        return;

                    int dir = 0;
                    if (m_keyForward == "Right") {
                        // update 4-switch
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

						if (keyEvent.isShiftPressed())
							modDir *= 10;

                        if (dir != 0 || modDir != 0) {
                            m_props[m_selectedProp] = m_props[m_selectedProp] + modDir;

                            viewer.updateText(" " + propertyNames[m_selectedProp] + ": " +
                                              std::to_string(m_props[m_selectedProp]), 200,
                                              maxY - (m_selectedProp + 1) * c_textHeight, 0, 1, 0,
                                              id2Str + "option " + std::to_string(m_selectedProp));
                            m_selectedProp = (m_selectedProp + propertyNames.size() + dir) % propertyNames.size();
                            viewer.updateText(">" + propertyNames[m_selectedProp] + ": " +
                                              std::to_string(m_props[m_selectedProp]), 200,
                                              maxY - (m_selectedProp + 1) * c_textHeight, 0, 1, 0,
                                              id2Str + "option " + std::to_string(m_selectedProp));
                            m_callee(m_currentState, m_props);
                        }
                    } else {
                        dir = 0;
                        if (keyEvent.getKeySym() == m_keyBackward)
                            dir = -1;
                        else if (keyEvent.getKeySym() == m_keyForward)
                            dir = +1;
                        else
                            return;

                        // update currently active text
                        viewer.updateText(
                                " " + m_states[m_currentState],
                                30, maxY - (m_currentState + 1) * c_textHeight,
                                0, 1, 0,
                                idStr + "option " + std::to_string(m_currentState));

                        m_currentState = (m_currentState + numStates + dir) % numStates;

                        // update now active text
                        viewer.updateText(
                                ">" + m_states[m_currentState],
                                30, maxY - (m_currentState + 1) * c_textHeight,
                                0, 1, 0,
                                idStr + "option " + std::to_string(m_currentState));
                        m_callee(m_currentState, m_props);
                    }
                });
    }
}

int SwitchControl::getState() {
    return m_currentState;
}
