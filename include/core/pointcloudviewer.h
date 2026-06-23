#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H

#include <QVTKOpenGLWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h> // 【必须包含】解决 'visualization' does not name a type
#include <QTimer>
#include <opencv2/core.hpp>

class PointCloudViewer : public QVTKOpenGLWidget
{
    Q_OBJECT
public:
    explicit PointCloudViewer(QWidget *parent = nullptr);
    ~PointCloudViewer();

    void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& id = "cloud");
    void showMesh(const pcl::PolygonMesh& mesh, const std::string& id = "mesh");
    void clearViewer();
    void resetCamera();
    void showCoordinateAxis(double scale = 100.0);
    void showRotationAxis(const cv::Mat& axisPoint, const cv::Mat& axisDirection, double scale);

private slots:
    void onTimerUpdate();

private:
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    QTimer* m_updateTimer;
};

#endif // POINTCLOUDVIEWER_H
