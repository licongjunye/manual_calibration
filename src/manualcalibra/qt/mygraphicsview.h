#ifndef MYGRAPHICSVIEW_H
#define MYGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QMouseEvent>
#include <QImage>
#include<QGraphicsEllipseItem>
#include <QScrollBar>
#include <QVBoxLayout>
#include<iostream>

class MyGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit MyGraphicsView(QWidget *parent = nullptr);
    ~MyGraphicsView();

    void setImage(const QImage &image);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
private:
    QImage m_image;
    QGraphicsScene *zoomScene;
    QGraphicsView *zoomView;
    QGraphicsScene *scene; // 新添加的场景
    QPoint m_mousePos;
    QPoint m_lastMouseClickPos;  // 新添加的变量，表示上一个鼠标点击的位置
    QVector<QLine> m_lines;
    QVector<QPoint> m_points;
    int m_clickCount;
    QList<QGraphicsEllipseItem *> m_pointItems;
    bool isCalibra = false;
    bool isscreenshot = false;

    QVector<QPoint> point;
    int emitcount = 0;

public:
    const QVector<QLine>& getLines() const {return m_lines;}
    void setGraphicsIscalibra(bool checked){isCalibra = checked;} 
    void setscreenshot(bool checked){isscreenshot = checked;} 

signals:
    void clickedAt(const QPointF &pos);
    void pointsSelected(const std::vector<QPoint> &points);
    void signalScreenshotPoint(const QVector<QPoint>);
    void signalScreenshotFinish(bool);
};

#endif // MYGRAPHICSVIEW_H
