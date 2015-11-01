/*
 * DrawBlock.h
 *
 *  Created on: Jun 26, 2013
 *      Author: aida
 */

#ifndef DRAWBLOCK_H_
#define DRAWBLOCK_H_

#include <Box2D/Box2D.h>
#include <QGraphicsScene>
#include "MakeAndBreak/Planner/Block.h"

class DrawBlock {
public:
	DrawBlock();
	DrawBlock(float x_, float y_, float w_, float h_, QGraphicsScene *scene_, b2World *world_,
			QColor color_, float scale, float offsetX_ = 0, float offsetY_ = 0);
	virtual ~DrawBlock();
	int width();
	int height();
	void display();
	void remove();

	b2Body *body;
	bool hide;

private:
	Block block;
	QColor color;

	QGraphicsScene *scene;
	b2World *world;
	QGraphicsRectItem *rect;
	float scale, offsetX, offsetY;
};

#endif /* DRAWBLOCK_H_ */
