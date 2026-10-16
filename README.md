# 双目线激光转台三维重建系统
Binocular Line Laser Turntable 3D Reconstruction — v1.0

> 用结构光（双目 + 线激光）与转台实现高精度小型工件三维扫描与重建。

![3D reconstruction demo](https://github.com/user-attachments/assets/9f8f6a57-28c1-459a-9b6f-d75056e420d2)

## 项目概览
- 目标：构建一套可复现的桌面级三维扫描与重建系统，适用于小型工件的逆向工程、尺寸测量与学术原型验证。
- 技术路线：双目视觉 + 线激光 + 旋转转台 + 多视角配准。
- 核心能力：高精度标定、激光条纹提取、三角测量、点云融合与表面重建。

## 核心亮点
- 完整标定流程：单目标定、双目标定、激光平面标定、转轴标定、系统标定
- 稳健重建管线：激光中心线提取、极线约束匹配、三角测量、坐标变换、ICP 配准、去噪与曲面重建
- 工业场景友好：Qt GUI、串口控制、结果自动保存至 `Resources/`
- 可扩展性：适合进一步开发自动扫描、异形件重建、视觉测量等功能

## 技术栈
- C++17
- CMake
- Qt5（GUI / 串口 / 并发 / OpenGL）
- OpenCV（图像处理、标定、立体匹配）
- PCL（点云处理、滤波、重建）
- Eigen3（线性代数与优化）
- VTK（3D 可视化）

## 快速开始
### Ubuntu / Debian
```bash
sudo apt install build-essential cmake \
    qt5-default libqt5opengl5-dev libqt5serialport5-dev \
    libopencv-dev libpcl-dev libeigen3-dev libvtk7-dev
```

### 构建
```bash
git clone <repo-url> 3D_reconstruction
cd 3D_reconstruction
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 运行
```bash
cd build
./3D_reconstruction
```

## 工作流说明
1. 双目采集：连接左右相机和串口，采集标定图像与扫描序列
2. 相机标定：完成单目和双目立体标定
3. 激光标定：提取激光中心线并拟合激光平面
4. 转轴标定：估计转台轴线与相机-转台位姿
5. 三维重建：运行 S1–S6 重建流水线，生成点云/网格结果

## 项目结构
```text
.
├── CMakeLists.txt
├── include/
│   ├── core/
│   ├── io/
│   └── ui/
├── src/
│   ├── core/
│   ├── io/
│   └── ui/
├── build/
│   └── Resources/
└── README.md
```

## 代表成果
- 单目标定 RMS：约 0.1 px 级
- 双目立体标定 RMS：约 0.16 px
- 激光平面拟合误差：约 0.27 mm
- 旋转轴标定误差：法向与轴夹角约 0.06°
- 重建点云有效率：约 98% 以上

## 适用场景
- 小型工件三维重建
- 逆向工程与几何建模
- 结构光视觉研究与教学演示
- 视觉测量与装配检测原型开发

## 贡献
欢迎提交 Issue 与 Pull Request，改进标定算法、交互体验或重建流程。
