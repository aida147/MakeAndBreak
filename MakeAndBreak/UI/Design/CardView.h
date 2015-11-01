/*
 * CardView.h
 *
 *  Created on: Jun 26, 2013
 *      Author: aida
 */

#ifndef CARDVIEW_H_
#define CARDVIEW_H_

#include "MakeAndBreak/Planner/Block.h"
#include "MakeAndBreak/Planner/Move.h"
#include "DrawBlock.h"
#include "MakeAndBreak/UI/MainWindow.h"

#include <vector>
#include <utility>

#include <QGraphicsView>
#include <Box2D/Box2D.h>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>

class CardView : public QGraphicsView
{
	Q_OBJECT
public:
	CardView(QWidget *parent = 0);
	virtual ~CardView();

	void reset();
	void start();
	void timerEvent(QTimerEvent *event);
	void setBlocks(std::vector< std::pair<Block, QColor> > a);
	void setBlocks(std::vector<Block> b, std::vector<QColor> c);
	void setRepeat(bool);
	void restart();

private:
	int timerId;
	b2World *world;
	std::vector<DrawBlock> dBlocks;
	std::vector<DrawBlock> ground;
	std::vector<Block> blocks;
	std::vector<QColor> colors;
	int counter;
	bool repeat;
};

class InfoPanel : public QGraphicsView
{
	Q_OBJECT
public:
	InfoPanel(QWidget *parent = 0);
	virtual ~InfoPanel();

	QLabel *messages;
	CardView *cv;
	QLabel *arrow;
	QGridLayout *layout;
};

#endif /* CARDVIEW_H_ */
