/*
 * Builder.h
 *
 *  Created on: Aug 6, 2013
 *      Author: aida
 */

#include "MakeAndBreak/UI/MainWindow.h"
#include "StepBox.h"
#include "RobotView.h"
#include "MakeAndBreak/UI/MakeAndBreakDefines.h"

#include <QtArmWidgets/QtImageCacheDisplay.h>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "Utilities/Debug.h"
#include "mab_msgs/MakeAndBreakOcsState.h"
#include "mab_msgs/MakeAndBreakOcsBehavior.h"
#include "mab_msgs/MakeAndBreakStepNumber.h"
#include "mab_msgs/MakeAndBreakInfo.h"

#ifndef BUILDER_H_
#define BUILDER_H_

class ControlButton;

class Builder : public QGraphicsView {
	Q_OBJECT
public:
	Builder(MainWindow *mw, QWidget *parent = 0);
	virtual ~Builder();

	void start();
	void buttonClicked(int index);
	void initTitle();
	void initButtons();
	void initSteps();
	void initRobot();
	void initCamera();
	void initTimer();
	void initInfo();
	void initLogo();

	MainWindow *mainWindow;
	std::vector<ControlButton *> controlButtons;
	QGridLayout *layout;
	std::vector<StepBox *> steps;
	RobotView *robotModel;
	QtImageCacheDisplay *imageDisplay;
	QStackedLayout *viewerLayout;
	QTimer *timer;
	QLabel *time;
	QLabel *description;
	int currentTime;

	ros::NodeHandle *m_nodeHandle;
	ros::Subscriber m_subState;
	ros::Subscriber m_subBehavior;
	MakeAndBreakOcsState ocs_state;
	ros::Subscriber m_subStep;
	ros::Subscriber m_subInfo;

	void updateInfo(const mab_msgs::MakeAndBreakInfo& msg);
	void initStateSwitch();

public slots:
	void updateTimer();
	void setAppState(const mab_msgs::MakeAndBreakOcsState& msg);
	void updateStepNumber(const mab_msgs::MakeAndBreakStepNumber& msg);
	void switchState();
};

class ControlButton : public QPushButton {
	Q_OBJECT
public:
	ControlButton(int index, Builder *parent);
	void mouseReleaseEvent(QMouseEvent *e);

	int index;
	Builder *parent;
};

#endif /* BUILDER_H_ */
