/*
 * MainWindow.h
 *
 *  Created on: Jul 1, 2013
 *      Author: aida
 */

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "MakeAndBreak/Planner/Block.h"
#include "MakeAndBreak/Planner/Move.h"
#include "MakeAndBreak/Planner/Solver.h"
#include "MakeAndBreak/UI/Design/Designer.h"
#include "MakeAndBreak/UI/Build/IdleView.h"
#include "MakeAndBreak/UI/Build/Builder.h"
#include "QtArmWidgets/QtEnvironmentRendererWidget.h"

#include <vector>
#include <iostream>
#include <sstream>

#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "MakeAndBreakDefines.h"

class Designer;
class IdleView;
class Builder;
class CameraDisplay;

struct blockMessage {
	int id;
	double angle;
	int relativeTo;
	bool orientation;
	std::pair<double, double> position;
	bool isValid;
	int colorR, colorG, colorB;
};

class MainWindow {
public:
	MainWindow(ros::Publisher*);
	virtual ~MainWindow();
	void init();
	void switchState();
	MakeAndBreakAppState getState();
	void mouseReleased();
	void mousePressed();
	void showWaiting();
	void hideWaiting();
	void readConf();

	bool isReady;
	std::vector< std::pair<blockMessage, blockMessage> > serviceData;
	std::vector<Block> blocks;
	std::vector<QColor> colors;
	std::vector<Move> moves;

private:
	void publishState();
	void setState(MakeAndBreakAppState state);
	void initRobot();

public:
	QWidget *main;

private:
	Designer *designer;
	IdleView *idleView;
	Builder *builder;
	QGraphicsView *waiting;
	QBoxLayout *layout;

	MakeAndBreakAppState m_state;
	ros::Publisher *m_statePub;
};

#endif /* MAINWINDOW_H_ */
