#include "core/pointcloudviewer.h"
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <QDebug>
#include <opencv2/core.hpp>
#include <cmath>

// ================= 唯一的构造函数 =================
PointCloudViewer::PointCloudViewer(QWidget *parent)
    : QVTKOpenGLWidget(parent)
{
    // 1. 正常创建 PCL Visualizer (它在后台会自动创建一个 vtkRenderer)
    m_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

    // 2. 把 PCL 内部创建的渲染器“偷”出来
    vtkSmartPointer<vtkRendererCollection> renderers = m_viewer->getRenderWindow()->GetRenderers();
    renderers->InitTraversal();
    vtkRenderer* pclRenderer = renderers->GetNextItem();

    // 3. 获取 Qt 控件的底层 VTK 渲染窗口
    vtkSmartPointer<vtkRenderWindow> qtRenderWindow = this->renderWindow();

    // 4. 【核心融合】：将 PCL 的渲染器，直接注入到 Qt 的渲染窗口中
    if (pclRenderer && qtRenderWindow) {
        qtRenderWindow->AddRenderer(pclRenderer);
    }

    // 5. 设置鼠标交互样式为轨道相机（解决左键平移问题）
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
        vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    if (qtRenderWindow->GetInteractor()) {
        qtRenderWindow->GetInteractor()->SetInteractorStyle(style);
        qtRenderWindow->GetInteractor()->Initialize();
    }

    // 6. 启动定时器，保证 3D 交互流畅不卡顿
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &PointCloudViewer::onTimerUpdate);
    m_updateTimer->start(100);
}

PointCloudViewer::~PointCloudViewer()
{
    if (m_updateTimer) {
        m_updateTimer->stop();
    }
}

void PointCloudViewer::onTimerUpdate()
{
    if (m_viewer) {
        m_viewer->spinOnce();
    }
}

void PointCloudViewer::clearViewer()
{
    if (m_viewer) {
        m_viewer->removeAllPointClouds();
        m_viewer->removeAllShapes();
        // 如果需要清空时连同坐标轴一起删除，可以取消下面这行的注释
        // m_viewer->removeCoordinateSystem("global_axis"); 
    }
}

void PointCloudViewer::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& id)
{
    if (!m_viewer || !cloud) return;

    if (m_viewer->contains(id)) {
        m_viewer->updatePointCloud(cloud, id);
    } else {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color(cloud, "z");
        m_viewer->addPointCloud(cloud, color, id);
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
        
        // 【新增】：首次添加点云时，自动显示坐标轴 (假设单位为mm，长度设为100.0)
        // showCoordinateAxis(10.0);
    }
}

void PointCloudViewer::showMesh(const pcl::PolygonMesh& mesh, const std::string& id)
{
    if (!m_viewer) return;

    try {
        if (m_viewer->contains(id)) {
            m_viewer->updatePolygonMesh(mesh, id);
        } else {
            m_viewer->addPolygonMesh(mesh, id);
        }
    } catch (const std::exception& e) {
        qDebug() << "显示网格失败:" << e.what();
    }
}

void PointCloudViewer::resetCamera()
{
    if (m_viewer) {
        m_viewer->resetCamera();
    }
}

// ================= 新增：显示坐标轴方法 =================
void PointCloudViewer::showCoordinateAxis(double scale)
{
    if (!m_viewer) return;
    
    std::string axis_id = "global_axis";
    // 防止重复添加坐标轴（通过固定ID判断）
    if (!m_viewer->contains(axis_id)) {
        // addCoordinateSystem(scale, id) 
        // scale: 坐标轴的长度，请根据你的点云实际尺寸调整（例如点云是毫米级的，可设为 50.0 或 100.0）
        m_viewer->addCoordinateSystem(scale, axis_id);
    }
}

// ================= 新增：显示旋转轴方法 =================
void PointCloudViewer::showRotationAxis(const cv::Mat& axisPoint, const cv::Mat& axisDirection, double scale)
{
    if (!m_viewer || axisPoint.empty() || axisDirection.empty()) return;

    // 1. 提取轴点坐标
    double px = axisPoint.at<double>(0);
    double py = axisPoint.at<double>(1);
    double pz = axisPoint.at<double>(2);

    // 2. 提取方向向量并归一化
    double dx = axisDirection.at<double>(0);
    double dy = axisDirection.at<double>(1);
    double dz = axisDirection.at<double>(2);
    
    double norm = sqrt(dx * dx + dy * dy + dz * dz);
    if (norm < 1e-6) {
        qDebug() << "旋转轴方向向量长度为0，无法绘制！";
        return;
    }
    dx /= norm;
    dy /= norm;
    dz /= norm;

    // 3. 计算直线的两个极远端点 (近似无限长直线)
    pcl::PointXYZ p1, p2;
    p1.x = px - dx * scale;
    p1.y = py - dy * scale;
    p1.z = pz - dz * scale;

    p2.x = px + dx * scale;
    p2.y = py + dy * scale;
    p2.z = pz + dz * scale;

    // 4. 移除旧轴线（防止重复点击重建时叠加）
    if (m_viewer->contains("rotation_axis_line")) {
        m_viewer->removeShape("rotation_axis_line");
    }
    if (m_viewer->contains("rotation_axis_center")) {
        m_viewer->removeShape("rotation_axis_center");
    }

    // 5. 添加红色直线表示旋转轴
    m_viewer->addLine(p1, p2, 1.0, 0.0, 0.0, "rotation_axis_line");
    // 设置线宽，使其更醒目
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "rotation_axis_line");

    // 6. 添加绿色小球表示旋转轴经过的物理点
    pcl::PointXYZ center;
    center.x = px;
    center.y = py;
    center.z = pz;
    m_viewer->addSphere(center, 2.0, 0.0, 1.0, 0.0, "rotation_axis_center");
}


