#ifndef IMAGEITEM_H
#define IMAGEITEM_H
#include <QGraphicsPixmapItem>
#include <QGraphicsRectItem>
#include <QObject>
#include <opencv2/opencv.hpp>
#include "imgpoint.h"
class ImageItem :public QObject,public QGraphicsPixmapItem
{
	Q_OBJECT
public:
    explicit ImageItem();

    bool LoadImg(QString path);
	bool LoadImg(const cv::Mat &img);
	void SetCorners(const std::vector<cv::Point2f> &corners, cv::Size pattern_size);
	void ClearCorners();
	bool GetCorners(std::vector<cv::Point2f> &corners);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

	void setSize(const QSize &size);
	QSize size() const;

    void AddNewRect(float x1, float y1, float x2, float y2);

private:
    QSize m_size;
	QGraphicsRectItem *m_rect;
	std::vector<ImgPoint*> m_corners;
	cv::Size m_pattern_size;
};

#endif // IMAGEITEM_H
