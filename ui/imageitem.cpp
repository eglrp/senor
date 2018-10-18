#include "imageitem.h"
#include <QDir>
#include <QFileInfo>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include <qdebug.h>
#include <string>
#include <Common_GQ.h>
#include <CornerFinder/CornerFinder.h>
#include <CameraCalibration/CameraBase.h>
using namespace std;
using namespace cv;
ImageItem::ImageItem():m_rect(nullptr)
{
}

bool ImageItem::LoadImg(QString path)
{  
    QFileInfo file(path);
    QPixmap img(file.absoluteFilePath());
	if (img.isNull())
		return false;
    m_size=img.size();
    setPixmap(img);
    return true;
}

bool ImageItem::LoadImg(const cv::Mat &img)
{
	if (img.empty())
		return false;
	QPixmap pix = QPixmap::fromImage(Common_GQ::cvMat2QImage(img));
	m_size = pix.size();
	setPixmap(pix);
	return true;
}

void ImageItem::SetCorners(const std::vector<cv::Point2f> &corners, cv::Size pattern_size)
{
	ClearCorners();
	QColor cl[] = { Qt::red,Qt::yellow,Qt::darkYellow,Qt::green,Qt::darkGreen,Qt::blue,Qt::darkBlue };
	for (size_t i = 0; i < corners.size(); i++)
	{
		ImgPoint* pt = new ImgPoint(this, scene());
		pt->setPos(QPointF(corners[i].x , corners[i].y));
		pt->setColor(cl[(i/ pattern_size.width)%7]);
		m_corners.push_back(pt);
	}
}

void ImageItem::ClearCorners()
{
	for (size_t i = 0; i < m_corners.size(); i++)
		delete m_corners[i];
	m_corners.clear();

}

bool ImageItem::GetCorners(std::vector<cv::Point2f> &corners)
{
	if (m_corners.size()==0)
	{
		return false;
	}
	corners.clear();
	for (size_t i = 0; i < m_corners.size(); i++)
	{
		QPointF &pt = m_corners[i]->pos();
		corners.push_back(cv::Point2f(pt.x(), pt.y()));
	}
	return true;
}

void ImageItem::setSize(const QSize &size)
{
    m_size = size;
}

QSize ImageItem::size() const
{
	return m_size;
}

void ImageItem::AddNewRect(float x1, float y1, float x2, float y2)
{
	if (!m_rect)
	{
		m_rect = new QGraphicsRectItem(this, scene());
		m_rect->setPen(QPen(Qt::green));
	}
	m_rect->setRect(x1, y1, x2-x1, y2-y1);
// 	Mat img = cv::imread(m_path.toLocal8Bit().constData(), 0);
// 	cv::Mat roi_img = img(cv::Rect(x1, y1, x2 - x1, y2 - y1)),temp;
// 	cv::Size pattern_size = cv::Size(9,10);
// 	int r=1;
// 	std::vector<cv::Point2f> corners;
// 	CornerFinder::FindCornersForOmniCamera(roi_img, pattern_size, corners);
// 	QColor cl[] = { Qt::red,Qt::yellow,Qt::darkYellow,Qt::green,Qt::darkGreen,Qt::blue,Qt::darkBlue };
// 	for (size_t i = 0; i < m_corners.size(); i++)
// 		delete m_corners[i];
// 	m_corners.clear();
// 	for (size_t i = 0; i < corners.size(); i++)
// 	{
// 		ImgPoint* pt = new ImgPoint(this, scene());
// 		pt->setPos(QPointF(corners[i].x + x1, corners[i].y + y1));
// 		pt->setColor(cl[(i/ pattern_size.width)%7]);
// 		m_corners.push_back(pt);
// 	}
}

void ImageItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsPixmapItem::mousePressEvent(event);
}

void ImageItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsPixmapItem::mouseMoveEvent(event);
}

void ImageItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}

