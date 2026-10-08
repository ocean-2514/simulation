#pragma once

#include <array>

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

    bool IsMouseButtonPressed(int button) const {
        if (button >= 0 && button < static_cast<int>(m_mouseButtons.size())) {
            return m_mouseButtons[button];
        }
        return false;
    }

private:
    std::array<bool, 512> m_keys = { false }; // Assuming a maximum of 512 keys
    std::array<bool, 16> m_mouseButtons = { false }; // Assuming a maximum of 16 mouse buttons
};