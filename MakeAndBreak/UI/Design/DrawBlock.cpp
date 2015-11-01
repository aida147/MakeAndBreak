/*
 * DrawBlock.cpp
 *
 *  Created on: Jun 26, 2013
 *      Author: aida
 */

#include "DrawBlock.h"
#include "MakeAndBreak/UI/UIConstants.h"

#include <cmath>
#include <Box2D/Box2D.h>
#include <QGraphicsRectItem>
#include <iostream>

DrawBlock::DrawBlock() : body(NULL), rect(NULL) {
	hide = false;
}

DrawBlock::~DrawBlock() {
}

DrawBlock::DrawBlock(float x, float y, float w, float h, QGraphicsScene *scene_,
		b2World *world_, QColor color_, float scale_, float offsetX_, float offsetY_)
: body(NULL), rect(NULL)
{
	hide = false;
	block = Block(x, y, w, h);
	scene = scene_;
	world = world_;
	color = color_;
	scale = scale_;
	offsetX = offsetX_;
	offsetY = offsetY_;

	b2FixtureDef fd;
	b2PolygonShape sd;
	sd.SetAsBox(block.width/2.0 - 0.01, block.height/2.0 - 0.01);
	fd.shape = &sd;
	fd.density = 25.0f;

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	float friction = .01f;

	fd.friction = friction;
	bd.position = b2Vec2(block.x, block.y);
	bd.angle = 0;
	body = world->CreateBody(&bd);
	body->CreateFixture(&fd);

	rect = scene->addRect(QRectF(
			(body->GetPosition().x - block.width/2.0) * scale + offsetX,
			(-body->GetPosition().y - block.height/2.0) * scale + offsetY,
			(block.width - 0.02) * scale, (block.height - 0.02) * scale));
	rect->hide();
}

void DrawBlock::display()
{
	if (hide) {
		rect->hide();
	} else {
		rect->setRect(QRectF(
				(body->GetPosition().x - block.width/2.0) * scale + offsetX,
				(-body->GetPosition().y - block.height/2.0) * scale + offsetY,
				block.width * scale,
				block.height * scale));
		rect->setTransformOriginPoint(body->GetPosition().x * scale + offsetX,
				-body->GetPosition().y * scale + offsetY);
		rect->setRotation(-(body->GetAngle() * 360.0) / (2 * M_PI));
		QBrush brush(color);
		rect->setBrush(brush);
		rect->setPen(Qt::NoPen);
		rect->show();
	}
}

void DrawBlock::remove()
{
	scene->removeItem(rect);
}
