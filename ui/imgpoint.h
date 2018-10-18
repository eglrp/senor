#ifndef IMGPOINT_H
#define IMGPOINT_H

#include <QGraphicsEllipseItem>
#include <QPen>

class ImgPoint : public QGraphicsEllipseItem
{

public:
    int type() const;

    explicit ImgPoint(QGraphicsItem *parent = 0,QGraphicsScene *scene = 0);
    ~ImgPoint();

	void setColor(QColor cl);
    void setPos(const QPointF &pos);
    QPointF pos() const;
    
    int Index();
    void SetIndex(int idx);
protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
private:
    QPen m_circle_pen;
    QPen m_line_pen;
    const int m_radius=5;
    QGraphicsLineItem *m_line_h;
    QGraphicsLineItem *m_line_v;
    int   index;  //该点在pointEditor中的索引
    
	QColor color;
    };

#endif // IMGPOINT_H
