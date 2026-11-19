#include "Eng.h"
#include "ClothSimulation.hpp"
#include <iostream>

int main() {
    Engine& engine = Engine::GetInstance();
    ClothSimulation stimulation;
    engine.SetApplication(&stimulation);
    engine.Init();
    engine.Run();
    return 0;
}