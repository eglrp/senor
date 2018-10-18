#include "imgpoint.h"

#include <QGraphicsSceneMouseEvent>
#include <QCursor>

int ImgPoint::type() const
{
    return 102;
}

ImgPoint::ImgPoint(QGraphicsItem *parent, QGraphicsScene *scene) : QGraphicsEllipseItem(parent,scene),m_line_h(NULL),m_line_v(NULL)
{
    setAcceptHoverEvents(true);
//     setFlags(GraphicsItemFlag::ItemIsMovable|GraphicsItemFlag::ItemIsSelectable);
    setZValue(100);
    m_circle_pen.setWidthF(1.0);
    m_line_pen.setWidthF(0.2);
    index = -1;
}

ImgPoint::~ImgPoint()
{
    if(m_line_h)
    {
        delete m_line_h;
        m_line_h=NULL;
    }
    if(m_line_v)
    {
        delete m_line_v;
        m_line_v=NULL;
    }
}

void ImgPoint::setColor(QColor cl)
{
	color = cl;
}

void ImgPoint::setPos(const QPointF &pos)
{
    QPointF pt=pos-QPointF(m_radius,m_radius);
    setRect(0,0,2*m_radius,2*m_radius);
    m_line_h=new QGraphicsLineItem(this,scene());
    m_line_v=new QGraphicsLineItem(this,scene());
    m_line_h->setLine(0,m_radius,2*m_radius,m_radius);
    m_line_v->setLine(m_radius,0,m_radius,2*m_radius);
    QGraphicsEllipseItem::setPos(pt);
}

QPointF ImgPoint::pos() const
{
    return QGraphicsEllipseItem::pos()+QPointF(m_radius,m_radius);
}

void ImgPoint::hoverEnterEvent(QGraphicsSceneHoverEvent *e)
{
   setCursor(Qt::SizeAllCursor);
    QGraphicsEllipseItem::hoverEnterEvent(e);
}

void ImgPoint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    m_circle_pen.setColor(color);
    m_line_pen.setColor(color);
    setPen(m_circle_pen);
    m_line_h->setPen(m_line_pen);
    m_line_v->setPen(m_line_pen);
    QGraphicsEllipseItem::paint(painter,option,widget);
}

void ImgPoint::SetIndex(int idx)
{
    index = idx;
}

int ImgPoint::Index()
{
    return index;
}