#include "Eng.h"
#include "Stimulation.hpp"
#include <iostream>

int main() {
    Engine& engine = Engine::GetInstance();
    Stimulation stimulation;
    engine.SetApplication(&stimulation);
    engine.Init();
    engine.Run();
    return 0;
}