#include "mygraphicsview.h"

#include <QDebug>

MyGraphicsView::MyGraphicsView(QWidget *parent)
    : QGraphicsView(parent) , m_clickCount(0)
{
    scene = new QGraphicsScene(this);
    setScene(scene);
    
    zoomScene = new QGraphicsScene(this);
    zoomView = new QGraphicsView(zoomScene);

    zoomView->setMinimumSize(200, 200);

    setMouseTracking(true); // 启用鼠标追踪
}

MyGraphicsView::~MyGraphicsView()
{
    delete zoomScene;
    delete zoomView;
}

void MyGraphicsView::setImage(const QImage &image)
{
    m_image = image;
    fitInView(scene->sceneRect());
}


void MyGraphicsView::mousePressEvent(QMouseEvent *event)
{
    if(isCalibra)
    {
        if(event->button() == Qt::LeftButton) 
        {
            QPoint imgPos = mapToScene(event->pos()).toPoint();

            if (imgPos.x() >= 0 && imgPos.y() >= 0 && imgPos.x() < m_image.width() && imgPos.y() < m_image.height())
            {
                emit clickedAt(imgPos);
            }
        }

        QGraphicsView::mousePressEvent(event);
    }else{
        zoomView->hide();
    }
    if(isscreenshot)
    {
        if(event->button() == Qt::LeftButton) 
        {
            QPoint imgPos = mapToScene(event->pos()).toPoint();
            
            if (imgPos.x() >= 0 && imgPos.y() >= 0 && imgPos.x() < m_image.width() && imgPos.y() < m_image.height())
            {
                point.clear();
                point.append(imgPos);
            }
        }

        QGraphicsView::mousePressEvent(event);
    }else{
        zoomView->hide();
    }

}
void MyGraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
    if (isscreenshot && event->button() == Qt::LeftButton) 
    {
        QPoint imgPos = mapToScene(event->pos()).toPoint();
        if (imgPos.x() >= 0 && imgPos.y() >= 0 && imgPos.x() < m_image.width() && imgPos.y() < m_image.height())
        {
            point.append(imgPos);
        }
        emit signalScreenshotPoint(point);
        emitcount++;
        qDebug()<<"emitcount:"<<emitcount<<endl;
        emit signalScreenshotFinish(false);
        isscreenshot = false;
        // std::cout<<"mouseReleaseEvent point: "<<point[0].x() <<" "<<point[0].y()<<" "<<point[1].x()<<" "<<point[1].y()<<std::endl;
        for(int i=0; i<point.size(); i++)
        {
            QPoint p = point[i];
            qDebug()<< "mouseReleaseEvent Point[" << i << "]: (" << p.x() << ", " << p.y() << ")";
            // this->logger->info( "mouseReleaseEvent:" + to_string(i) + ": "+  to_string(p.x()) + "," + to_string(p.y()));
            // this->logger->info( "mouseReleaseEvent:");
        }
    }
    QGraphicsView::mouseReleaseEvent(event);
}
void MyGraphicsView::mouseMoveEvent(QMouseEvent *event)
{
    if(isCalibra || isscreenshot)
    {
        QGraphicsView::mouseMoveEvent(event);

        // 将鼠标位置转换为图像的坐标
        QPoint imgPos = mapToScene(event->pos()).toPoint();

        // 检查坐标是否在图像范围内
        if (imgPos.x() >= 0 && imgPos.y() >= 0 && imgPos.x() < m_image.width() && imgPos.y() < m_image.height())
        {
            // 获取放大的区域（例如，鼠标周围的100x100像素区域）
            int zoomSize = 100;
            QRect zoomRect(imgPos.x() - zoomSize/2, imgPos.y() - zoomSize/2, zoomSize, zoomSize);
            
            // 确保区域在图像范围内
            zoomRect = zoomRect.intersected(m_image.rect());
            
            // 获取该区域的图像
            QImage zoomImage = m_image.copy(zoomRect);
            
            // 将图像放大（例如，放大2倍）
            zoomImage = zoomImage.scaled(zoomSize * 2, zoomSize * 2);
            
            // 在放大视图中显示图像
            QPixmap pixmap = QPixmap::fromImage(zoomImage);
            zoomScene->clear();
            zoomScene->addPixmap(pixmap);

            // 在zoomView正中添加一个绿色的点
            QPen pen(Qt::green);
            pen.setWidth(2);
            zoomScene->addEllipse(QRectF(zoomSize - 1, zoomSize - 1, 2, 2), pen);
            
            zoomView->setSceneRect(0, 0, zoomImage.width(), zoomImage.height());
            zoomView->fitInView(zoomView->sceneRect(), Qt::KeepAspectRatio);
            // zoomView->setWindowFlags(Qt::WindowStaysOnTopHint);
            zoomView->show();
        }
    }else{
        zoomView->hide();
    }
}

