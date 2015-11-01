/*
 * Draggable.h
 *
 *  Created on: Jul 1, 2013
 *      Author: aida
 */

#ifndef DRAGABLE_H_
#define DRAGABLE_H_

#include <QGraphicsRectItem>

class Dragable : public QGraphicsRectItem{
public:
	Dragable(double x, double y, double w, double h, QList<QGraphicsItem *> il,
			QColor color, QRectF workspace_);
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);
	void mouseReleaseEvent (QGraphicsSceneMouseEvent * event);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event);
	bool checkForCollision(QPointF newPos);
	void changeOrientation();
	QPointF snap(QPointF newPos);

	QList<QGraphicsItem *> ignoreList;
	QString error;
	bool isCurrentError;
	QRectF workspace;

private:
	QPointF orig;
};

#endif /* DRAGABLE_H_ */
