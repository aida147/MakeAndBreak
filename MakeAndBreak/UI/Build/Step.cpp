/*
 * Step.cpp
 *
 *  Created on: Jul 26, 2013
 *      Author: aida
 */

#include "Step.h"

Step::Step(std::vector<Block> blocks, std::vector<QColor> colors, QGraphicsScene *scene, float offsetX, float offsetY) :
blocks(blocks), colors(colors), scene(scene), offsetX(offsetX), offsetY(offsetY)
{
	state = NEXT;
	rects.resize(0);
	draw();
}

Step::~Step()
{
	// Empty
}

QColor toGray(QColor c)
{
	float g = (c.red() * 11 + c.green() * 16 + c.blue() * 5)/32;
	return QColor(g, g, g);
}

void Step::draw()
{
	for (int i = 0; i < (int) rects.size(); i++)
		scene->removeItem(rects[i]);
	rects.resize(0);

	for (int i = 0; i < (int) blocks.size(); i++)
		if (state == PAST)
			rects.push_back(scene->addRect(QRectF(
					(blocks[i].x - blocks[i].width/2.0) * UI::STEPS_BLOCK_WIDTH + offsetX,
					(9 - blocks[i].y - blocks[i].height/2.0) * UI::STEPS_BLOCK_WIDTH + offsetY,
					blocks[i].width * UI::STEPS_BLOCK_WIDTH, blocks[i].height * UI::STEPS_BLOCK_WIDTH),
					QPen(toGray(colors[i])), QBrush(toGray(colors[i]))));
		else
			rects.push_back(scene->addRect(QRectF(
					(blocks[i].x - blocks[i].width/2.0) * UI::STEPS_BLOCK_WIDTH + offsetX,
					(9 - blocks[i].y - blocks[i].height/2.0) * UI::STEPS_BLOCK_WIDTH + offsetY,
					blocks[i].width * UI::STEPS_BLOCK_WIDTH, blocks[i].height * UI::STEPS_BLOCK_WIDTH),
					QPen(colors[i]), QBrush(colors[i])));

	if (state == CURRENT)
		rects.push_back(scene->addRect(offsetX, offsetY,
				UI::STEP_BOX_WIDTH,
				UI::STEP_BOX_HEIGHT,
				QPen(UI::LIGHT_GREEN, 5)));
	else
		rects.push_back(scene->addRect(offsetX, offsetY,
				UI::STEP_BOX_WIDTH,
				UI::STEP_BOX_HEIGHT,
				QPen(UI::LIGHT_GRAY)));

}

void Step::setState(int s)
{
	state = s;
	draw();
}
