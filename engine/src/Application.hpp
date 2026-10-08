#pragma once

class Application
{
public:
    virtual ~Application() = default;
    virtual bool Init() = 0;
    virtual void Update(float deltaTime) = 0;
};