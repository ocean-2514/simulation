# 构建与运行

## 环境要求

- Windows。
- CMake 3.10 或更高版本。
- 支持 C++17 的 MinGW-w64 GCC 工具链，以及 `mingw32-make`。
- 支持 OpenGL 3.3 Core Profile 的显卡驱动。

仓库已包含 GLAD、GLFW/GLM 头文件、`engine/lib/libglfw3dll.a` 和 `bin/glfw3.dll`。当前 CMake 配置使用 MinGW 的 `.a` 导入库和 Windows GLFW DLL；MSVC 或其他平台尚未配置。

构建过程中不会从网络下载依赖。请使用与仓库中 GLFW 导入库兼容的 MinGW-w64 工具链；GLFW DLL 需要在运行时与可执行文件位于同一目录。窗口创建还依赖本机显卡驱动提供 OpenGL 3.3 Core Profile。

## 配置和编译

从仓库根目录打开 PowerShell：

```powershell
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build
```

`cmake -S . -B build` 以仓库根目录的 `CMakeLists.txt` 配置两个主要目标：`engine` 静态库和 `simulation` 演示程序。`engine` 中包含项目代码和 GLAD；项目目标通过 `engine/lib/libglfw3dll.a` 链接 GLFW。可执行文件输出到 `build/bin/simulation.exe`，构建后步骤会将仓库中的 GLFW DLL 复制到同一目录。

单独构建引擎目标可使用：

```powershell
cmake --build build --target engine
```

若 `build` 目录以前用其他 CMake generator 配置过，应使用新的构建目录配置，或先通过 CMake 重新生成该目录。CMake generator 在同一构建目录内不能直接切换。

## 运行

```powershell
.\build\bin\simulation.exe
```

程序会创建 OpenGL 3.3 窗口并运行当前 `Stimulation` 场景。使用 W/A/S/D 控制运动学三角形，按 Esc 关闭窗口。场景中的刚体、碰撞边界和约束由 `include/Stimulation.hpp` 创建。

当前编译定义将着色器目录设置为源代码树中的 `src/shader` 绝对路径，所以运行时需要保留完整源代码目录；移动或复制可执行文件到独立目录不会自动打包着色器。

## 构建配置和依赖位置

| 项目 | 当前配置 |
|---|---|
| C++ 标准 | C++17 |
| CMake 最低版本 | 3.10 |
| GLFW 头文件 | `engine/include/GLFW/` |
| GLFW MinGW 导入库 | `engine/lib/libglfw3dll.a` |
| GLFW 运行时 | `bin/glfw3.dll`，构建后复制到 `build/bin/` |
| GLAD | `engine/include/glad/` 与 `engine/include/KHR/`，作为 `glad` 目标编译 |
| GLM | `engine/include/glm/` |
| Shader 源码 | `src/shader/` |

GLFW、GLAD、GLM 都随仓库提供头文件或二进制依赖，因此首次构建不需要运行 package manager。若替换 GLFW 版本，需要同步更新头文件、MinGW 导入库和 DLL。

## 常见问题

- **CMake 找不到 MinGW Makefiles 构建器**：确认 MinGW-w64 和 `mingw32-make` 已加入 `PATH`，并重新配置项目。
- **找不到 GLFW 导入库**：检查 `engine/lib/libglfw3dll.a` 是否存在。
- **启动时提示缺少 GLFW DLL**：确认 `build/bin/glfw3.dll` 存在；CMake 的 post-build 步骤会从 `bin/glfw3.dll` 复制它。
- **窗口无法创建或 GLAD 初始化失败**：确认显卡驱动支持 OpenGL 3.3 Core Profile。
- **着色器文件加载失败**：当前构建把仓库 `src/shader` 路径写入程序；从完整的源代码目录配置和构建项目。
- **链接时报导入库格式错误**：检查当前 C++ 编译器是否为兼容的 MinGW-w64，而不是 MSVC；本仓库提供的是 MinGW `.a` 导入库。
- **窗口大小或显示比例不合预期**：当前窗口尺寸在 `Engine` 构造函数的默认参数中设定，演示相机投影根据窗口宽高比计算。

项目目前没有单独的测试目标。`cmake --build` 用于编译引擎和演示程序，不代表已运行物理回归测试。
