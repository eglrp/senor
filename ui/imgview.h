#ifndef IMGVIEW_H
#define IMGVIEW_H

#include <QGraphicsView>
#include <QListWidgetItem>
#include "imageitem.h"
#include <opencv2/opencv.hpp>
class ImgView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit ImgView(QWidget *parent = 0);
    void wheelEvent(QWheelEvent *event);
    void LoadImg(QString path);
	void LoadImg(const cv::Mat &img);
	void ShowCorners(const std::vector<cv::Point2f> &corners, cv::Size pattern_size);

	void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
	QSize ImgSize() const;

    void Clear();
signals:

private:
    QGraphicsScene *m_scene;
    ImageItem *m_img_item;
    QGraphicsRectItem *m_rect;
};

#endif // IMGVIEW_H
