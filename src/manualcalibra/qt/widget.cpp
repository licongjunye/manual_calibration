#include "widget.h"
#include "ui_widget.h"
#include <QImage>
#include <QPixmap>


Widget::Widget(int argc, char** argv, QWidget *parent)
    : QWidget(parent), ui(new Ui::Widget), argc(argc), argv(argv), timer(new QTimer(this)), m_scene(0), m_image_item(0)
{
    ui->setupUi(this);

    ros::init(argc, argv, "manualcalibra");

    if (!read_param(K_0, C_0, E_0))
    {
        QMessageBox::information(NULL, "info", "fail to read camera.yaml");
    }else{
        // std::cout << "K_0: " << K_0 << std::endl;
        // std::cout << "C_0: " << C_0 << std::endl;
        // std::cout << "E_0: " << E_0 << std::endl;
    }



    ros::NodeHandle nh;
    // clickpoint_sub = nh.subscribe("/initialpose", 1, &Widget::initialPoseCallback, this);
    clickpoint_sub = nh.subscribe("/clicked_point", 1, &Widget::clickedPointCallback, this);
   
    lidar_thread = std::thread(std::bind(&Widget::spinloop,this));
    


    m_scene = new QGraphicsScene(this);
    m_image_item = new QGraphicsPixmapItem();
    m_scene->addItem(m_image_item);
    ui->graphicsView->setScene(m_scene);

    QStringList camera_headers;
    camera_headers << "X" << "Y";
    ui->tableWidget_camera->setColumnCount(2);
    ui->tableWidget_camera->setHorizontalHeaderLabels(camera_headers);
    ui->tableWidget_camera->horizontalHeader()->setStretchLastSection(true);

    QStringList lidar_headers;
    lidar_headers << "X" << "Y" << "Z";
    ui->tableWidget_lidar->setColumnCount(3);
    ui->tableWidget_lidar->setHorizontalHeaderLabels(lidar_headers);
    ui->tableWidget_lidar->horizontalHeader()->setStretchLastSection(true);


    QObject::connect(ui->graphicsView,SIGNAL(clickedAt(QPointF)),this,SLOT(handleClickAt(QPointF)));


    // cap.open("/home/hlc/code/RM_Radar_file/fenqu/20240531_143420_2min40s.mp4");
    cap.open("/home/hlc/code/RM_Radar_file/gongxun/0728/20240801 060823.mp4");

    if (!cap.isOpened()) {
        qDebug("Failed to open video file.");
        return;
    }



    connect(timer, &QTimer::timeout, this, &Widget::updateFrame);
    timer->start(33); // 大约30帧每秒
}

Widget::~Widget()
{
    delete ui;
    cap.release();
}

void Widget::spinloop()
{
    while(ros::ok())
    {
        ros::spinOnce();
    }
}

bool Widget::read_param(Mat &K_0, Mat &C_0, Mat &E_0)
{
       // 检查文件是否存在
    if (access(CAMERA_PARAM_PATH, F_OK) != 0)
    {
        std::cerr << "File does not exist: " << CAMERA_PARAM_PATH << std::endl;
        return false;
    }

    // 打开文件
    cv::FileStorage fs(CAMERA_PARAM_PATH, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open file: " << CAMERA_PARAM_PATH << std::endl;
        return false;
    }

    // 读取参数
    try
    {
        fs["K_0"] >> K_0;
        fs["C_0"] >> C_0;
        fs["E_0"] >> E_0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading parameters from file: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void Widget::updateFrame()
{
    cv::Mat frame;
    if (cap.read(frame)) {
        // 将 BGR 格式的帧转换为 RGB 格式
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        if(camera_pick_points.size() != 0)
        {
            for(const auto &it : camera_pick_points)
            {
                circle(frame, it, 1, Scalar(0,255,0), 4);
            }
        }

        QImage qimg = cvMatToQImage(frame);
        
         ui->graphicsView->setImage(qimg);
        if(m_image_item)
        {
            m_scene->removeItem(m_image_item);
            delete m_image_item;
            m_image_item = 0;
        }

        m_image_item = m_scene->addPixmap(QPixmap::fromImage(qimg));
        m_scene->setSceneRect(0, 0, qimg.width(), qimg.height());
        ui->graphicsView->viewport()->update();
    } else {
        timer->stop();
    }
}

QImage Widget::cvMatToQImage(const cv::Mat &mat)
{
    switch (mat.type()) {
        // 8-bit, 4 channel
        case CV_8UC4:
            return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_ARGB32);

        // 8-bit, 3 channel
        case CV_8UC3:
            return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888);

        // 8-bit, 1 channel
        case CV_8UC1:
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
            return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8);
#else
            vector<QRgb> colorTable;
            for (int i = 0; i < 256; i++)
                colorTable.push_back(qRgb(i, i, i));
            QImage img(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Indexed8);
            img.setColorTable(colorTable);
            return img;
#endif

        default:
            qDebug("cvMatToQImage: Mat image type not handled in switch: %d", mat.type());
            break;
    }

    return QImage();
}

void Widget::on_checkBox_pickpoints_clicked(bool checked)
{
    ui->graphicsView->setGraphicsIscalibra(checked);
}

void Widget::handleClickAt(QPointF pos)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    camera_pick_points.emplace_back(cv::Point2f(pos.rx(), pos.ry()));

    if(camera_pick_points.size() != 0)
    {
        // 清空现有表格内容
        ui->tableWidget_camera->clearContents();
        ui->tableWidget_camera->setRowCount(0);

        // 填充新的表格内容
        int row = 0;
        for (const auto& point : camera_pick_points) {
            ui->tableWidget_camera->insertRow(row);
            ui->tableWidget_camera->setItem(row, 0, new QTableWidgetItem(QString::number(point.x)));
            ui->tableWidget_camera->setItem(row, 1, new QTableWidgetItem(QString::number(point.y)));
            ++row;
        }
    }
}

void Widget::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::cout << "x: " << msg->point.x << " y: " << msg->point.y << " z: " << msg->point.z << std::endl;
    lidar_pick_points.emplace_back(cv::Point3f(msg->point.x, msg->point.y, msg->point.z));

    if(lidar_pick_points.size() != 0)
    {
        // 清空现有表格内容并重置行数
        ui->tableWidget_lidar->clearContents();
        ui->tableWidget_lidar->setRowCount(0);

        // 填充新的表格内容
        int row = 0;
        for (const auto& point : lidar_pick_points) {
            ui->tableWidget_lidar->insertRow(row);
            ui->tableWidget_lidar->setItem(row, 0, new QTableWidgetItem(QString::number(point.x)));
            ui->tableWidget_lidar->setItem(row, 1, new QTableWidgetItem(QString::number(point.y)));
            ui->tableWidget_lidar->setItem(row, 2, new QTableWidgetItem(QString::number(point.z)));
            ++row;
        }
    }
}

void Widget::on_pushButtonsolvepnp_clicked(bool checked)
{
if(camera_pick_points.size() != lidar_pick_points.size())
    {
        QMessageBox::information(NULL,"info", "camera_pick_points.size() != lidar_pick_points.size()");
        return;
    }

    Mat rvec_Mat, tvec_Mat;
    if (!solvePnP(lidar_pick_points, camera_pick_points, K_0, C_0, rvec_Mat, tvec_Mat, false, SolvePnPMethod::SOLVEPNP_ITERATIVE))
    {
        QMessageBox::information(NULL, "info", "fail to solvepnp");
    }else{
        std::cout<<"\n"<<"\n"<<"\n"<<"\n"<<std::endl;
        std::cout << "rvec_Mat: " << rvec_Mat << std::endl;
        std::cout << "tvec_Mat: " << tvec_Mat << std::endl;

        // 将旋转向量转换为旋转矩阵
        cv::Mat R;
        cv::Rodrigues(rvec_Mat, R);

        // 创建一个4x4单位矩阵
        cv::Mat E = cv::Mat::eye(4, 4, CV_64F);

        // 复制旋转矩阵到变换矩阵的左上角
        R.copyTo(E(cv::Rect(0, 0, 3, 3)));

        // 复制平移向量到变换矩阵的最后一列
        tvec_Mat.copyTo(E(cv::Rect(3, 0, 1, 3)));

        // 设置输出格式保留六位小数
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "E: " << E << std::endl;
    }
}



void Widget::on_pushButton_deletecamerapoint_clicked(bool checked)
{
    if (!camera_pick_points.empty()) {
        camera_pick_points.pop_back();  // 删除最后一个元素
    }
    
    if(camera_pick_points.size() != 0)
    {
        // 清空现有表格内容
        ui->tableWidget_camera->clearContents();
        ui->tableWidget_camera->setRowCount(0);

        // 填充新的表格内容
        int row = 0;
        for (const auto& point : camera_pick_points) {
            ui->tableWidget_camera->insertRow(row);
            ui->tableWidget_camera->setItem(row, 0, new QTableWidgetItem(QString::number(point.x)));
            ui->tableWidget_camera->setItem(row, 1, new QTableWidgetItem(QString::number(point.y)));
            ++row;
        }
    }
}


void Widget::on_pushButton_deletelidarpoint_clicked(bool checked)
{
    if(lidar_pick_points.size() != 0)
    {
        lidar_pick_points.pop_back();
    }

    if(lidar_pick_points.size() != 0)
    {
        // 清空现有表格内容并重置行数
        ui->tableWidget_lidar->clearContents();
        ui->tableWidget_lidar->setRowCount(0);

        // 填充新的表格内容
        int row = 0;
        for (const auto& point : lidar_pick_points) {
            ui->tableWidget_lidar->insertRow(row);
            ui->tableWidget_lidar->setItem(row, 0, new QTableWidgetItem(QString::number(point.x)));
            ui->tableWidget_lidar->setItem(row, 1, new QTableWidgetItem(QString::number(point.y)));
            ui->tableWidget_lidar->setItem(row, 2, new QTableWidgetItem(QString::number(point.z)));
            ++row;
        }
    }
}