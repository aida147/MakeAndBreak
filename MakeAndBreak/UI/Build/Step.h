/*
 * Step.h
 *
 *  Created on: Jul 26, 2013
 *      Author: aida
 */

#ifndef STEP_H_
#define STEP_H_

#include "MakeAndBreak/Planner/Block.h"
#include "MakeAndBreak/UI/UIConstants.h"

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <vector>

class Step {
	std::vector<Block> blocks;
	std::vector<QColor> colors;
	std::vector<QGraphicsItem *> rects;
	QGraphicsScene *scene;
	float offsetX, offsetY;
	int state;

public:
	Step(std::vector<Block> blocks, std::vector<QColor> colors, QGraphicsScene *scene, float offsetX, float offsetY);
	void draw();
	void setState(int s);
	virtual ~Step();

	enum {
		PAST = 0,
		CURRENT,
		NEXT
	};
};


#endif /* STEP_H_ */
