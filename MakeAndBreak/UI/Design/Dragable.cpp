/*
 * Dragable.cpp
 *
 *  Created on: Jul 1, 2013
 *      Author: aida
 */

#include "Dragable.h"
#include "MakeAndBreak/UI/UIConstants.h"

#include <QGraphicsScene>
#include <iostream>
#include <QGraphicsItem>

Dragable::Dragable(double x, double y, double w, double h,
		QList<QGraphicsItem *> il, QColor color, QRectF workspace_)
:QGraphicsRectItem(x + 0.01, y + 0.01, w - 0.02, h - 0.02)
{
	setFlag(QGraphicsItem::ItemIsSelectable);
	setFlag(QGraphicsItem::ItemSendsGeometryChanges);
	setFlag(QGraphicsItem::ItemIsMovable);
	QBrush brush(color);
	setBrush(brush);
	setPen(QPen(UI::LIGHT_GRAY));

	orig = this->pos();
	ignoreList = il;
	isCurrentError = false;
	workspace = workspace_;
}

bool equ(double x, double y)
{
	return x-y > 0 ? x-y < 1e-6 : y-x < 1e-6;
}

void Dragable::changeOrientation()
{
	QPointF p = this->boundingRect().topLeft();
	QRectF cur = this->rect();
	QPointF n = QPointF(cur.center().x() - cur.height() / 2.0,
			cur.center().y() - cur.width() / 2.0);
	this->setRect(n.x(), n.y(), this->rect().height(), this->rect().width());

	if (checkForCollision(this->pos())) {
		this->setRect(cur);
		error = "Can't rotate here. The block will collide\nwith other blocks or boundary lines.";
		isCurrentError = true;
	}
}

void Dragable::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event)
{
	changeOrientation();
}

bool Dragable::checkForCollision(QPointF newPos)
{
	QRectF p = this->boundingRect();
	double left = this->mapToScene(p.bottomLeft()).x(),
			right = this->mapToScene(p.bottomRight()).x(),
			top = this->mapToScene(p.topRight()).y(),
			bottom = this->mapToScene(p.bottomRight()).y();

	if (right > workspace.right() || left < workspace.left() ||
			bottom > workspace.bottom() || top < workspace.top()) {
		error = "Don't move blocks too far from the grid.";
		isCurrentError = true;
		this->setPos(orig);
		return true;
	}

	QList<QGraphicsItem *> l = scene()->items();
	for (int i = 0; i < l.length(); i++) {
		QRectF r = l[i]->boundingRect();
		bool dontCheck = false;
		for (int j = 0; j < ignoreList.size(); j++) {
			if (l[i] == ignoreList[j]) {
				dontCheck = true;
				break;
			}
		}

		if (dontCheck)
			continue;

		if ((l[i] != this && l[i]->collidesWithItem(this))) {
			error = "Can't move here. The block will collide\nwith other blocks or boundary lines.";
			isCurrentError = true;
			this->setPos(orig);
			return true;
		}
	}

	orig = newPos;
	return false;
}

void Dragable::mouseReleaseEvent(QGraphicsSceneMouseEvent * event)
{
	QGraphicsItem::mouseReleaseEvent(event);
	QPointF newPos = snap(this->pos());
	this->setPos(newPos);
	if (!checkForCollision(newPos))
		this->setPos(newPos);
	this->setSelected(false);
}

QPointF Dragable::snap(QPointF newPos)
{
	int scale = UI::MAIN_BLOCK_WIDTH;
	{
		if ((int)newPos.x() % scale < (scale/2))
			newPos.setX(newPos.x() - (int)newPos.x()%scale);
		else
			newPos.setX(newPos.x() - (int)newPos.x()%scale + scale);
	}
	{
		if ((int)newPos.y() % scale < (scale/2))
			newPos.setY(newPos.y() - (int)newPos.y()%scale);
		else
			newPos.setY(newPos.y() - (int)newPos.y()%scale + scale);
	}
	return newPos;
}

QVariant Dragable::itemChange(GraphicsItemChange change, const QVariant &value)
{
/*	if (change == ItemPositionChange && scene()) {
		QPointF newPos = snap(value.toPointF());
		if (newPos != value.toPointF())
			return newPos;
	}*/
	return QGraphicsItem::itemChange(change, value);
}
