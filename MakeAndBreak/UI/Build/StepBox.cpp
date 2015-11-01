/*
 * StepBox.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: aida
 */

#include "StepBox.h"
#include "MakeAndBreak/UI/UIConstants.h"

StepBox::StepBox(std::vector<Block> blocks, std::vector<QColor> colors, int blockWidth, QWidget *parent) :
QGraphicsView(parent), blocks(blocks), colors(colors), blockWidth(blockWidth), state(NEXT)
{
	QGraphicsScene *sc = new QGraphicsScene();
	sc->setBackgroundBrush(Qt::transparent);
	sc->setSceneRect(-10, -10, 6 * blockWidth + 20, 9 * blockWidth + 20);
	setScene(sc);
	UI::init(this, false);
	this->setMinimumSize(6 * blockWidth + 20, 9 * blockWidth + 20);

	rects.resize(0);
	draw();
}

StepBox::~StepBox() {
	// TODO Auto-generated destructor stub
}

QColor grayscale(QColor c)
{
	float g = (c.red() * 11 + c.green() * 16 + c.blue() * 5)/32;
	return QColor(g, g, g);
}

void StepBox::draw()
{
	for (int i = 0; i < (int) rects.size(); i++)
		scene()->removeItem(rects[i]);
	rects.resize(0);

	if (state == CURRENT)
		rects.push_back(scene()->addRect(-10, -10, 6 * blockWidth + 20, 9 * blockWidth + 20,
				QPen(QColor(139, 188, 94)), QBrush(QColor(0, 187, 36))));

	for (int i = 0; i < (int) blocks.size(); i++)
		if (state == PAST)
			rects.push_back(scene()->addRect(QRectF(
					(blocks[i].x - blocks[i].width/2.0) * blockWidth,
					(9 - blocks[i].y - blocks[i].height/2.0) * blockWidth,
					blocks[i].width * blockWidth, blocks[i].height * blockWidth),
					QPen(UI::LIGHT_GRAY.darker(300)), QBrush(colors[i].darker(300))));
		else
			rects.push_back(scene()->addRect(QRectF(
					(blocks[i].x - blocks[i].width/2.0) * blockWidth,
					(9 - blocks[i].y - blocks[i].height/2.0) * blockWidth,
					blocks[i].width * blockWidth, blocks[i].height * blockWidth),
					QPen(UI::LIGHT_GRAY), QBrush(colors[i])));
	switch (state) {
	case PAST:
		scene()->addLine(-5, 9 * blockWidth,
			6 * blockWidth + 10, 9 * blockWidth, QPen(UI::LIGHT_GRAY.darker(300), 3));
		break;
	case CURRENT:
		scene()->addLine(-5, 9 * blockWidth,
			6 * blockWidth + 10, 9 * blockWidth, QPen(Qt::black, 3));
		break;
	default:
		scene()->addLine(-5, 9 * blockWidth,
			6 * blockWidth + 10, 9 * blockWidth, QPen(UI::LIGHT_GRAY, 3));
	}
}

void StepBox::setState(int s)
{
	state = s;
	draw();
}

Seperator::Seperator(bool hasArrow, int height, QWidget *parent) :
QGraphicsView(parent)
{
	QGraphicsScene *sc = new QGraphicsScene();
	sc->setBackgroundBrush(Qt::transparent);
	sc->setSceneRect(-18, 0, 40, height);
	setScene(sc);
	UI::init(this, false);
	this->setMinimumSize(40, height);

	sc->addLine(0, 0, 0, height, QPen(QColor(100, 149, 199), 3));
	if (hasArrow) {
		QGraphicsPixmapItem * arrow =
				new QGraphicsPixmapItem(QPixmap("./data/makeAndBreak/BlueNext.png"));
		arrow->setPos(-18, height / 2 - 20);
		sc->addItem(arrow);
	}
}
