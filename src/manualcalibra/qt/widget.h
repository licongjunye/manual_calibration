#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include"mygraphicsview.h"
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <mutex>
#include<vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <thread>
#include<QMessageBox>
#include <fstream>
#include <iomanip>


#define CAMERA_PARAM_PATH (char *)"/home/hlc/code/RM_Radar2024/lidar_base/src/radar2023/RadarClass/Camera/params/camera0.yaml" // 相机参数文件 SJUT or hometest


using namespace cv;
using namespace std;

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(int argc, char** argv, QWidget *parent = nullptr);
    ~Widget();

    // void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    
    void spinloop();
    bool read_param(Mat &K_0, Mat &C_0, Mat &E_0);


private slots:
    void updateFrame();
    void on_checkBox_pickpoints_clicked(bool checked);
    void handleClickAt(QPointF);
    void on_pushButtonsolvepnp_clicked(bool checked);
    void on_pushButton_deletecamerapoint_clicked(bool checked);
    void on_pushButton_deletelidarpoint_clicked(bool checked);

private:
    Ui::Widget *ui;

    int argc; // 命令行参数的数量
    char **argv; // 命令行参数的指针数组 

    ros::Subscriber clickpoint_sub;
    

    cv::VideoCapture cap;
    QTimer *timer;
    QImage cvMatToQImage(const cv::Mat &mat);
    QGraphicsScene *m_scene;
    QGraphicsPixmapItem *m_image_item;

    vector<Point2f> camera_pick_points;
    vector<Point3f> lidar_pick_points;
    mutable std::mutex m_mutex;
    std::thread lidar_thread;
    Mat K_0, C_0, E_0;

};
#endif // WIDGET_H
