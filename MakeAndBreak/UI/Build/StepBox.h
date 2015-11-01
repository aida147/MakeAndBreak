/*
 * StepBox.h
 *
 *  Created on: Aug 7, 2013
 *      Author: aida
 */

#ifndef STEPBOX_H_
#define STEPBOX_H_

#include "MakeAndBreak/Planner/Block.h"
#include "MakeAndBreak/UI/UIConstants.h"

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <vector>

class StepBox : public QGraphicsView {
	Q_OBJECT
public:
	StepBox(std::vector<Block> blocks, std::vector<QColor> colors, int blockWidth, QWidget *parent = 0);
	virtual ~StepBox();
	void draw();
	void setState(int s);

	std::vector<Block> blocks;
	std::vector<QColor> colors;
	std::vector<QGraphicsItem *> rects;
	int blockWidth;
	int state;

	enum {
		PAST = 0,
		CURRENT,
		NEXT
	};
};

class Seperator : public QGraphicsView {
	Q_OBJECT
public:
	Seperator(bool hasArrow, int height, QWidget *parent = 0);
//	virtual ~Seperator();
};

#endif /* STEPBOX_H_ */
