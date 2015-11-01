/*
 * BuildingArea.h
 *
 *  Created on: Jul 31, 2013
 *      Author: aida
 */

#ifndef BUILDINGAREA_H_
#define BUILDINGAREA_H_

#include <QGraphicsView>

#include "MakeAndBreak/Planner/Block.h"
#include "Dragable.h"
#include "Designer.h"

#include <vector>

class Designer;

class BuildingArea : public QGraphicsView {
	Q_OBJECT
public:
	BuildingArea(Designer *designer, QWidget *parent = 0);
	virtual ~BuildingArea();
	std::vector< std::pair<Block, QColor> > getPattern();
	void addBlocks(QList<QGraphicsItem *> &ignoreList);
	void addGrid(QList<QGraphicsItem *> &ignoreList);
	void mouseReleaseEvent(QMouseEvent *event);
	void setActive(bool state);

	std::vector<Dragable *> blocks;
	Designer *designer;
};

#endif /* BUILDINGAREA_H_ */
