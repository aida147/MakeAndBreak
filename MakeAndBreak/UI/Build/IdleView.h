/*
 * IdleView.h
 *
 *  Created on: Jul 8, 2013
 *      Author: aida
 */

#ifndef IDLEVIEW_H_
#define IDLEVIEW_H_

#include "MakeAndBreak/UI/MainWindow.h"
#include "MakeAndBreak/UI/MakeAndBreakDefines.h"
#include "MakeAndBreak/Planner/Move.h"
#include "Step.h"
#include "CameraDisplay.h"
#include "RobotView.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "Utilities/Debug.h"
#include "mab_msgs/MakeAndBreakOcsState.h"
#include "mab_msgs/MakeAndBreakOcsBehavior.h"
#include "mab_msgs/MakeAndBreakStepNumber.h"

#include <QObject>
#include <QGraphicsScene>
#include <Box2D/Box2D.h>
#include <QPushButton>
#include <QSignalMapper>

#include <utility>
#include <vector>
#include <string>

class MainWindow;
class CameraButton;
class CameraDisplay;

class IdleView : public QGraphicsView
{
	Q_OBJECT
public:
	virtual ~IdleView();
	IdleView(MainWindow *mw, QWidget *parent = 0);
	void setAppState(const mab_msgs::MakeAndBreakOcsState& msg);
	void start();
	void addLabels();
	void updateCommands(const mab_msgs::MakeAndBreakOcsBehavior& msg);
	void addCommand(std::string s);
	void loadImage(int i);

private slots:
	void handleButton();

protected:
	void timerEvent(QTimerEvent *event);

private:
	void init();
	QString toMinutesAndSeconds(int time);
	void initLabels();
	void initCommands();
	void initSteps();
	void initCameraAndTimer();
	void initRobot();
	void updateStepNumber(const mab_msgs::MakeAndBreakStepNumber& msg);

	enum {
		RIGHT = 0,
		LEFT,
		NECK
	};

	int timerId;
	QGraphicsScene *scene;
	MainWindow *mainWindow;
	QLabel *timer;
	QLabel *commands;
	int counter;
	ros::NodeHandle *m_nodeHandle;
	ros::Subscriber m_subState;
	ros::Subscriber m_subBehavior;
	MakeAndBreakOcsState ocs_state;
	std::vector<Step> steps;
	ros::Subscriber m_subStep;
	CameraDisplay *camera;
	QGridLayout *layout;
	RobotView *robotModel;

};

#endif /* IDLEVIEW_H_ */
