# 双目线激光转台三维重建系统
Binocular Line Laser Turntable 3D Reconstruction — v1.0

简介
基于双目视觉、线激光与旋转转台的结构光三维扫描与重建系统。两台同步相机从不同视角采集激光条纹，结合转台旋转，通过标定、立体匹配与三角测量生成高质量点云并进行配准与重建。

主要特性
- 五步标定：单目标定 → 双目标定 → 激光平面标定 → 转轴标定 → 系统标定
- 六阶段重建流水线（S1–S6）：光条提取、极线匹配、三角测量、坐标变换、ICP 多视角配准、去噪与曲面重建
- 基于 OpenCV / PCL / Eigen / Qt 的 C++ 实现，支持可视化与串口控制
- 标定与重建结果自动保存至 Resources/

技术栈
- C++17, CMake
- Qt5（GUI、并发、串口、OpenGL）
- OpenCV (≥4.4), PCL (≥1.12), Eigen3, VTK

快速开始（Ubuntu）
1. 依赖安装（示例）
   sudo apt install build-essential cmake qt5-default libqt5opengl5-dev libqt5serialport5-dev libopencv-dev libpcl-dev libeigen3-dev libvtk7-dev
2. 构建
   git clone <repo-url> 3D_reconstruction
   cd 3D_reconstruction
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
3. 运行
   cd build
   ./3D_reconstruction

运行流程
1. 双目采集 — 连接相机与串口，采集标定/扫描图像  
2. 相机标定 — 单目与双目立体标定  
3. 激光标定 — 激光中心线提取与平面拟合  
4. 转轴标定 — 旋转轴方向与位姿估计  
5. 三维重建 — 配置 S1–S6 参数并执行重建

项目结构
- CMakeLists.txt
- include/ — 头文件（cameracalibration, lasercalibration, pointcloudbuilder, rotatingcalibrator, …）
- src/ — 源码（main, core, io, ui）
- build/Resources/ — 运行数据与标定结果
