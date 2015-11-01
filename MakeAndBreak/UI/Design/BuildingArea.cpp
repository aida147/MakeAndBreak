/*
 * BuildingArea.cpp
 *
 *  Created on: Jul 31, 2013
 *      Author: aida
 */

#include "BuildingArea.h"
#include "MakeAndBreak/UI/UIConstants.h"
#include <QGraphicsLineItem>
#include <QMouseEvent>

BuildingArea::BuildingArea(Designer *designer, QWidget *parent) :
QGraphicsView(parent), designer(designer)
{
	QGraphicsScene *s = new QGraphicsScene();
	s->setSceneRect(0, 0, UI::BUILDING_WIDTH, UI::BUILDING_HEIGHT);
	setScene(s);
	UI::init(this, false);

	QList<QGraphicsItem *> ignoreList;
	addGrid(ignoreList);
	addBlocks(ignoreList);

}

BuildingArea::~BuildingArea() {
	// TODO Auto-generated destructor stub
}

int convertForSnap(float x)
{
	return (int) x - (int) x % (int) UI::MAIN_BLOCK_WIDTH;
}

void BuildingArea::addGrid(QList<QGraphicsItem *> &ignoreList)
{
	for (int i = 0; i <= UI::GRID_COLUMNS; i++) {
		QGraphicsLineItem * t = scene()->addLine(
				convertForSnap(UI::GRID_LEFT + i * UI::MAIN_BLOCK_WIDTH),
				convertForSnap(UI::GRID_BOTTOM),
				convertForSnap(UI::GRID_LEFT + i * UI::MAIN_BLOCK_WIDTH),
				convertForSnap(UI::GRID_TOP),
				QPen(UI::LIGHT_GRAY));

		if (i != 0 && i != UI::GRID_COLUMNS)
			ignoreList.push_back(t);
	}

	for (int i = 0; i <= UI::GRID_ROWS; i++) {
		QGraphicsLineItem * t = scene()->addLine(
				convertForSnap(UI::GRID_LEFT),
				convertForSnap(UI::GRID_TOP + i * UI::MAIN_BLOCK_WIDTH),
				convertForSnap(UI::GRID_RIGHT),
				convertForSnap(UI::GRID_TOP + i * UI::MAIN_BLOCK_WIDTH),
				QPen(UI::LIGHT_GRAY));

		if (i != 0 && i != UI::GRID_ROWS)
			ignoreList.push_back(t);
	}
}

void BuildingArea::addBlocks(QList<QGraphicsItem *> &ignoreList)
{
	QRectF designingArea(0, 0, UI::BUILDING_WIDTH, UI::BUILDING_HEIGHT);
	int colorIndex = 0;
	for (int i = 0; i < 3; i++)	// blocks on top
		blocks.push_back(new Dragable(
				convertForSnap(UI::GRID_LEFT + (-2 + i * 4) * UI::MAIN_BLOCK_WIDTH),
				convertForSnap(UI::GRID_TOP / 2.0f - 0.5 * UI::MAIN_BLOCK_WIDTH),
				UI::MAIN_BLOCK_LENGTH,
				UI::MAIN_BLOCK_WIDTH,
				ignoreList,
				UI::list[colorIndex++],
				designingArea));

	for (int i = 0; i < 2; i++)	// blocks on left
		blocks.push_back(new Dragable(
				convertForSnap(UI::GRID_LEFT - 2 * UI::MAIN_BLOCK_WIDTH),
				convertForSnap(UI::GRID_TOP + (i * 5) * UI::MAIN_BLOCK_WIDTH),
				UI::MAIN_BLOCK_WIDTH,
				UI::MAIN_BLOCK_LENGTH,
				ignoreList,
				UI::list[colorIndex++],
				designingArea));

	for (int i = 0; i < 2; i++)	// blocks on right
		blocks.push_back(new Dragable(
				convertForSnap(UI::GRID_LEFT + (6 + 1) * UI::MAIN_BLOCK_WIDTH),
				convertForSnap(UI::GRID_TOP + (i * 5) * UI::MAIN_BLOCK_WIDTH),
				UI::MAIN_BLOCK_WIDTH,
				UI::MAIN_BLOCK_LENGTH,
				ignoreList,
				UI::list[colorIndex++],
				designingArea));
	for (int i = 0; i < (int) blocks.size(); i++)
		scene()->addItem(blocks[i]);
}

std::vector< std::pair<Block, QColor> > BuildingArea::getPattern()
{
	std::vector< std::pair<Block, QColor> > pat;
	pat.resize(0);
	for (int i = 0; i < (int) blocks.size(); i++) {
		Dragable *db = blocks[i];
		QPointF p = db->mapToScene(db->boundingRect().center());

		if (p.x() < convertForSnap(UI::GRID_LEFT) || p.x() > convertForSnap(UI::GRID_RIGHT) ||
				p.y() < convertForSnap(UI::GRID_TOP) || p.y() > convertForSnap(UI::GRID_BOTTOM))
			continue;

		p-= QPointF(convertForSnap(UI::GRID_LEFT), convertForSnap(UI::GRID_BOTTOM));
		p/= UI::MAIN_BLOCK_WIDTH;
		if (db->boundingRect().width() > db->boundingRect().height())
			pat.push_back(std::pair<Block, QColor>(
					Block(p.x(), -p.y(), UI::BLOCK_LENGTH, UI::BLOCK_WIDTH),
					db->brush().color()));
		else
			pat.push_back(std::pair<Block, QColor>(
					Block(p.x(), -p.y(), UI::BLOCK_WIDTH, UI::BLOCK_LENGTH),
					db->brush().color()));
	}

	return pat;
}

void BuildingArea::mouseReleaseEvent(QMouseEvent *event)
{
	QGraphicsView::mouseReleaseEvent(event);
	designer->mainWindow->mouseReleased();
}

void BuildingArea::setActive(bool state)
{
	for (int i = 0; i < (int) blocks.size(); i++) {
		blocks[i]->setFlag(QGraphicsItem::ItemIsMovable, state);
		blocks[i]->setFlag(QGraphicsItem::ItemIsSelectable, state);
	}
}
