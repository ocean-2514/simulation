#pragma once

class Application
{
public:
    virtual ~Application() = default;
    virtual bool Init() = 0;
    virtual void Update(float deltaTime) = 0;

    void SetShouldStop(bool stop) { m_shouldStop = stop; }
    bool ShouldStop() const { return m_shouldStop; }

private:
    bool m_shouldStop = false;
};