/*
 * Designer.h
 *
 *  Created on: Jun 27, 2013
 *      Author: aida
 */

#ifndef DESIGNER_H_
#define DESIGNER_H_

#include "MakeAndBreak/UI/MainWindow.h"
#include "CardView.h"
#include "BuildingArea.h"

#include <utility>
#include <vector>

#include <QObject>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsScene>
#include <QPushButton>
#include <QGraphicsPixmapItem>
#include <QLabel>
#include <QGridLayout>

class MainWindow;
class CardView;
class BuildingArea;
class InfoPanel;

class Designer : public QGraphicsView
{
	Q_OBJECT
public:
	virtual ~Designer();
	Designer(MainWindow *mw, QWidget *parent = 0);
	void start();
	std::vector< std::pair<Block, QColor> > getPattern();
	void updateCV();
	void addMessage(QString message, bool showArrow = false);
	void clearMessage();
	void stop();

private slots:
    void handleButton();

private:
    void initButton();
    void initInstructions();
	void convert();

    QPushButton *buildButton;
    QLabel *label;
    QLabel *instructions;
    InfoPanel *info;
    QGridLayout *layout;

public:
    MainWindow *mainWindow;
private:
    QLabel *logo;
	int timerId;
public:
	int messageTimer;
	BuildingArea *buildingArea;
	bool stopped;

protected:
	void timerEvent(QTimerEvent *event);
};

#endif /* DESIGNER_H_ */
