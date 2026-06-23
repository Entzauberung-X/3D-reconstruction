## 构建与运行

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./camera
```

## 源文件结构

```
src/
├── main.cpp                  # 入口 + 全局QSS主题
├── ui/
│   ├── mainwindow.cpp        # 构造骨架 + 菜单/工具栏 + 共享工具 + 自动加载标定
│   ├── mainwindow_tab1.cpp   # Tab1 采集 UI + 15槽函数
│   ├── mainwindow_tab2.cpp   # Tab2 标定 UI + 合并加载按钮 + 标定持久化
│   ├── mainwindow_tab3.cpp   # Tab3 激光 UI + 13槽函数
│   ├── mainwindow_tab4.cpp   # Tab4 转台 UI + 12槽函数
│   ├── mainwindow_tab5.cpp   # Tab5 重建 UI + 8槽函数
│   ├── mainwindow_roi.cpp    # ROI框选共享辅助
│   ├── calibration_io.cpp    # 标定参数YAML序列化/反序列化 + 加载后参数报告弹窗
│   └── videowidget.cpp       # OpenCV→Qt 视频渲染控件
├── core/
│   ├── pointcloudbuilder.cpp # S1-S6核心算法
│   ├── cameracalibration.cpp # 单/双目标定
│   ├── lasercalibration.cpp  # 光平面标定
│   ├── rotatingcalibrator.cpp# 旋转轴自动标定
│   ├── pointcalibrator.cpp   # SIFT单点测距
│   └── pointcloudviewer.cpp  # PCL 3D可视化
└── io/
    ├── camerathread.cpp      # 相机采集线程
    ├── laserworker.cpp       # 激光处理
    └── serialportmanager.cpp # 串口管理
include/
├── ui/
│   ├── mainwindow.h          # 类定义
│   ├── theme.h               # 集中式主题常量
│   ├── logger.h              # 分级日志系统
│   └── videowidget.h
├── core/
│   ├── pointcloudbuilder.h
│   ├── cameracalibration.h
│   ├── lasercalibration.h
│   ├── rotatingcalibrator.h
│   ├── pointcalibrator.h
│   └── pointcloudviewer.h
└── io/
    ├── camerathread.h
    ├── laserworker.h
    └── serialportmanager.h
```

