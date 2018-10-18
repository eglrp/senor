#include "imgview.h"
#include <QScrollBar>
#include <QWheelEvent>
#include <QDebug>

ImgView::ImgView(QWidget *parent) : QGraphicsView(parent)
{
    m_scene=new QGraphicsScene(this);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setRenderHint(QPainter::Antialiasing, false);
    setOptimizationFlags(QGraphicsView::DontSavePainterState);
    setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setScene(m_scene);
    m_img_item=NULL;
    m_rect=NULL;
}

void ImgView::wheelEvent(QWheelEvent *event)
{
//     int radius=1;
//     double kx=size().width()/(double)(m_img_item->size().width()+radius),ky=size().height()/(double)(m_img_item->size().height()+radius);
//     double min_scale=(kx<ky?kx:ky);
//     if(transform().m11()<=min_scale&&event->delta()<0)
//         return;

    double zoom_factor;
    const QPointF p0scene = mapToScene(event->pos());
    if(event->delta()>0)
        zoom_factor=1.1;
    else
        zoom_factor=1/1.1;
    scale(zoom_factor,zoom_factor);
//     if(transform().m11()<=min_scale)
//     {
//         double k=min_scale/transform().m11();
//         setTransform(transform().scale(k,k));
//     }
//     const QPointF p1mouse = mapFromScene(p0scene);
//     const QPointF move = p1mouse - event->pos(); // The move
//     horizontalScrollBar()->setValue(move.x() + horizontalScrollBar()->value());
//     verticalScrollBar()->setValue(move.y() + verticalScrollBar()->value());
//     m_scene->update();;
}

void ImgView::LoadImg(QString path)
{
    if(m_img_item)
        delete m_img_item;
    m_img_item=new ImageItem;
    m_img_item->LoadImg(path);
    int radius=10;
    m_scene->setSceneRect(0,0,m_img_item->size().width()+radius,m_img_item->size().height()+radius);
    m_scene->addItem(m_img_item);
    m_img_item->setPos(radius/2,radius/2);
    m_scene->setBackgroundBrush(QBrush(Qt::white));
//     double kx=size().width()/(double)(m_scene->sceneRect().width()+radius),ky=size().height()/(double)(m_scene->sceneRect().height()+radius);
//     double min_scale=(kx<ky?kx:ky);
//     QTransform tf;
//     tf.setMatrix(min_scale,0,0,0,min_scale,0,0,0,1);
//     setTransform(tf);
//     centerOn(m_img_item);
}

void ImgView::LoadImg(const cv::Mat &img)
{
	if (m_img_item)
		delete m_img_item;
	m_img_item = new ImageItem;
	m_img_item->LoadImg(img);
	int radius = 10;
	m_scene->setSceneRect(0, 0, m_img_item->size().width() + radius, m_img_item->size().height() + radius);
	m_scene->addItem(m_img_item);
	m_img_item->setPos(radius / 2, radius / 2);
	m_scene->setBackgroundBrush(QBrush(Qt::white));

}

void ImgView::ShowCorners(const std::vector<cv::Point2f> &corners, cv::Size pattern_size)
{
	m_img_item->SetCorners(corners, pattern_size);
}

void ImgView::mousePressEvent(QMouseEvent *event)
{
    QPoint pos=event->pos();
    if(!m_img_item)
        return;
    if(event->button()==Qt::RightButton)
    {
		QGraphicsItem *item=this->itemAt(pos);
        if(item&&item->type()==7)            //µã»÷pixmapitem¿Õ°×´¦
        {
            if(m_rect)
            {
                delete m_rect;
            }
            m_rect=new QGraphicsRectItem(m_img_item,m_scene);
            QPen pen(Qt::yellow);
            pen.setWidth(2);
            m_rect->setPen(pen);
            m_rect->setPos(mapToScene(pos));
        }
        else if(this->itemAt(pos)->type()==20)
        {
            delete this->itemAt(pos);
        }
    }
    QGraphicsView::mousePressEvent(event);
}

void ImgView::mouseMoveEvent(QMouseEvent *event)
{
    if(m_rect)
    {
        QPointF pos=mapToScene(event->pos());
        QPointF temp(pos-m_rect->pos());
        if(temp.x()+pos.x())
            m_rect->setRect(0,0,temp.x(),temp.y());
    }
    QGraphicsView::mouseMoveEvent(event);
}

void ImgView::mouseReleaseEvent(QMouseEvent *event)
{
    if(m_rect)
    {
        if(m_img_item)
        {
            QPointF p1=m_rect->pos(),p2=m_rect->rect().bottomRight()+p1;
            if(p1.x()<p2.x()&&p1.y()<p2.y())
            {
                m_img_item->AddNewRect(p1.x(),p1.y(),p2.x(),p2.y());
            }
            else
            {
                delete m_rect;
                m_rect=0;
                return;
            }
        }
        delete m_rect;
        m_rect=0;
    }
    this->update();
    QGraphicsView::mouseReleaseEvent(event);
}

QSize ImgView::ImgSize() const
{
	return m_img_item->size();
}

void ImgView::Clear()
{
    if(m_img_item)
    {
        delete m_img_item;
        m_img_item=0;
    }
}


