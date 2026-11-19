#pragma once

#include <array>

#include "Input/Key.h"
#include <glm/glm.hpp>

class InputManager {
public:
    InputManager() = default;

    void SetKeyState(int key, bool isPressed) {
        if (key >= 0 && key < static_cast<int>(m_keys.size())) {
            m_keys[key] = isPressed;
        }
    }

    void SetMouseButtonState(int button, bool isPressed) {
        if (button >= 0 && button < static_cast<int>(m_mouseButtons.size())) {
            m_mouseButtons[button] = isPressed;
        }
    }

    bool IsKeyPressed(int key) const {
        if (key >= 0 && key < static_cast<int>(m_keys.size())) {
            return m_keys[key];
        }
        return false;
    }

    bool IsKeyPressed(Key key) const {
        return IsKeyPressed(static_cast<int>(key));
    }

    bool IsMouseButtonPressed(int button) const {
        if (button >= 0 && button < static_cast<int>(m_mouseButtons.size())) {
            return m_mouseButtons[button];
        }
        return false;
    }

    void SetMousePos(float xpos, float ypos) {
        if (m_firstMouse) {
            m_firstMouse = false;
            m_prevMousePos = m_mousePos = {xpos, ypos};
            return;
        }
        m_mousePos = {xpos, ypos};
    }

    glm::vec2 GetMousePosOffset() const {
        return m_mousePos - m_prevMousePos;
    }

    void Clear() {
        m_prevMousePos = m_mousePos;
    }

private:
    std::array<bool, 512> m_keys = { false }; // Assuming a maximum of 512 keys
    std::array<bool, 16> m_mouseButtons = { false }; // Assuming a maximum of 16 mouse buttons

    glm::vec2 m_mousePos{0.0f};
    glm::vec2 m_prevMousePos{0.0f};
    bool m_firstMouse = true;
};