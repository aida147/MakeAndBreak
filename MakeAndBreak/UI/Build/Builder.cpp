/*
 * Builder.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: aida
 */

#include "Builder.h"
#include "MakeAndBreak/UI/UIConstants.h"
#include <string>
#include <iostream>

Builder::Builder(MainWindow *mw, QWidget *parent) :
QGraphicsView(parent), mainWindow(mw)
{
	m_nodeHandle = new ros::NodeHandle();
	m_subState = m_nodeHandle->subscribe("MakeAndBreakOcsState", 1000, &Builder::setAppState, this);
	ocs_state = MAKE_BREAK_OCS_STATE_BUILD;
	m_subStep = m_nodeHandle->subscribe("MakeAndBreakStepNumber", 1000, &Builder::updateStepNumber, this);
	m_subInfo = m_nodeHandle->subscribe("MakeAndBreakInfo", 1000, &Builder::updateInfo, this);

	QGraphicsScene *sc = new QGraphicsScene();
	sc->setSceneRect(0, 0, UI::WIDTH, UI::HEIGHT);
	sc->setBackgroundBrush(QBrush(QColor(4, 11, 41)));
	setScene(sc);

	layout = new QGridLayout();
	setLayout(layout);

	initTitle();
	initButtons();
	initSteps();
	initRobot();
	initCamera();
	initTimer();
	initInfo();
	initLogo();

	viewerLayout = new QStackedLayout();
	viewerLayout->addWidget(robotModel);
	viewerLayout->addWidget(imageDisplay);
	layout->addLayout(viewerLayout, 2, 1, 1, 1);

	controlButtons[0]->click();
	buttonClicked(0);

	initStateSwitch();
}

Builder::~Builder()
{
	SAFE_DELETE(m_nodeHandle);
}

void Builder::initStateSwitch()
{
	QTimer *t = new QTimer(this);
	t->setInterval(100);
	t->setSingleShot(false);
	connect(t, SIGNAL(timeout()), this, SLOT(switchState()));
}

void Builder::switchState()
{
	if (ocs_state == MAKE_BREAK_OCS_STATE_IDLE
			&& mainWindow->getState() == MAKE_BREAK_APP_STATE_IDLE)
		mainWindow->switchState();
}

void Builder::start()
{
	timer->start();
}

void Builder::initTitle()
{
	QLabel *missionControl = new QLabel();
	missionControl->setText("MISSION CONTROL");
	missionControl->setFont(UI::BOLD_FONT);
	missionControl->setStyleSheet("color : rgb(252, 252, 55)");
	layout->addWidget(missionControl, 0, 1, 1, 1, Qt::AlignCenter);
}

void Builder::initButtons()
{
	QGridLayout *l = new QGridLayout();
	l->setVerticalSpacing(0);

	controlButtons.resize(3);
	for (int i = 0; i < 3; i++) {
		controlButtons[i] = new ControlButton(i, this);
		l->addWidget(controlButtons[i], 0, i, 1, 1);
	}
	controlButtons[0]->setText("Robot\nViewer");
	controlButtons[1]->setText("Color\nCamera");
	controlButtons[2]->setText("Depth\nCamera");

	layout->addLayout(l, 1, 1, 1, 1);
}

void Builder::initSteps()
{
	QFrame *borders[2];
	for (int i = 0; i < 2; i++) {
		borders[i] = new QFrame();
		borders[i]->setFrameStyle(QFrame::HLine | QFrame::Plain);
		borders[i]->setStyleSheet("QFrame { border: 2px solid rgb(236, 17, 0); }");
	}
	layout->addWidget(borders[0], 4, 0, 1, -1);
	layout->addWidget(borders[1], 6, 0, 1, -1);

	QHBoxLayout *l = new QHBoxLayout();
	l->setSpacing(0);

	steps.resize(0);
	std::vector<Block> bt;
	std::vector<QColor> ct;

	l->addWidget(new Seperator(false, UI::STEPS_BLOCK_WIDTH * 10), 1, Qt::AlignCenter);

	for (int i = 0; i < (int) mainWindow->moves.size(); i++) {
		if (mainWindow->moves[i].right != Move::NO_BLOCK) {
			bt.push_back(mainWindow->blocks[mainWindow->moves[i].right]);
			ct.push_back(mainWindow->colors[mainWindow->moves[i].right]);
		}
		if (mainWindow->moves[i].left != Move::NO_BLOCK) {
			bt.push_back(mainWindow->blocks[mainWindow->moves[i].left]);
			ct.push_back(mainWindow->colors[mainWindow->moves[i].left]);
		}

		StepBox *step = new StepBox(bt, ct, UI::STEPS_BLOCK_WIDTH);
		steps.push_back(step);
		l->addWidget(step, 1, Qt::AlignBottom | Qt::AlignHCenter);

		if (i+1 != (int) mainWindow->moves.size()) {
			l->addWidget(new Seperator(true, UI::STEPS_BLOCK_WIDTH * 10), 1, Qt::AlignCenter);
		}
	}

	l->addWidget(new Seperator(false, UI::STEPS_BLOCK_WIDTH * 10), 1, Qt::AlignCenter);

	layout->addLayout(l, 5, 0, 1, 4, Qt::AlignCenter);
}

void Builder::initRobot()
{
	robotModel = new RobotView(QColor(4, 11, 41));
}

void Builder::initCamera()
{
	imageDisplay = new QtImageCacheDisplay();
}

void Builder::initTimer()
{
	QHBoxLayout *l = new QHBoxLayout();
	QLabel *label = new QLabel();
	label->setText("Timer:");
	label->setFont(UI::BOLD_FONT);
	label->setStyleSheet("color : white;");
	l->addWidget(label);

	time = new QLabel();
	time->setText("00:00");
	time->setFont(UI::BOLD_FONT);
	time->setStyleSheet("color : rgb(225, 105, 21);");
	l->addWidget(time);
	layout->addLayout(l, 3, 1, 1, 1, Qt::AlignCenter);

	timer = new QTimer(this);
	timer->setInterval(1000);
	timer->setSingleShot(false);
	connect(timer, SIGNAL(timeout()), this, SLOT(updateTimer()));
	currentTime = 0;
}

void Builder::updateTimer()
{
	currentTime+= 1;
	std::string s = "01:34";
	s[4] = '0' + currentTime % 10;
	s[3] = '0' + (currentTime / 10) % 6;
	s[1] = '0' + (currentTime / 60) % 10;
	s[0] = '0' + (currentTime / 600) % 6;
	time->setText(s.c_str());
}

void Builder::initInfo()
{
	QGraphicsView *info = new QGraphicsView();
	QGraphicsScene *sc = new QGraphicsScene();
	sc->setBackgroundBrush(Qt::transparent);
	info->setScene(sc);
	UI::init(info, false);
	info->setMinimumSize(805, 405);
	sc->addRect(0, 0, 800, 400, QPen(UI::LIGHT_BLUE), QBrush(QColor(49, 66, 96)));

	QGridLayout *l = new QGridLayout();
	description = new QLabel(info);
	description->setPixmap(QPixmap("./data/makeAndBreak/robot.png"));
	l->addWidget(description, 0, 0, 1, 1, Qt::AlignCenter);
	info->setLayout(l);
	layout->addWidget(info, 1, 2, 3, 2, Qt::AlignTop | Qt::AlignLeft);
}

void Builder::initLogo()
{
	QLabel *logo = new QLabel();
	logo->setPixmap(QPixmap("./data/makeAndBreak/RI_logo.png"));
	layout->addWidget(logo, 0, 3, 1, 1, Qt::AlignBottom | Qt::AlignRight);
}

void Builder::buttonClicked(int index)
{
	for (int i = 0; i < 3; i++)
		if (i != index)
			controlButtons[i]->setChecked(false);

	switch (index) {
	case 0:	// Robot Viewer
		viewerLayout->setCurrentIndex(0);
		break;
	case 1:	// Color Camera
		imageDisplay->setImageName("Kinect/RGB");
		viewerLayout->setCurrentIndex(1);
		break;
	case 2:	// Depth Camera
		imageDisplay->setImageName("Kinect/DepthGray");
		viewerLayout->setCurrentIndex(1);
		break;
	}
}

void Builder::setAppState(const mab_msgs::MakeAndBreakOcsState& msg)
{
	ARMS_INFO("M&B OCS State : " << msg.state);
	ocs_state = static_cast<MakeAndBreakOcsState>(msg.state);
}

void Builder::updateStepNumber(const mab_msgs::MakeAndBreakStepNumber& msg)
{
	ARMS_INFO("M&B Step : " << msg.step);
	int i = msg.step;
	if (i < (int) steps.size())
		steps[i]->setState(StepBox::CURRENT);
	if (i - 1 >= 0 && i - 1 < (int) steps.size())
		steps[i - 1]->setState(StepBox::PAST);
}

void Builder::updateInfo(const mab_msgs::MakeAndBreakInfo& msg)
{
	ARMS_INFO("M&B load image : " << msg.name);
	description->setPixmap(QPixmap(("./data/makeAndBreak/" + msg.name).c_str()));
}

ControlButton::ControlButton(int index, Builder *parent) :
QPushButton(), index(index), parent(parent)
{
	std::string style = "QPushButton { \
			border: 2px solid #64a1e0; \
				background-color: #64a1e0; \
				color: #ffffff;\
				font-weight: bold;";

	switch (index) {
	case 0:
		style+= "border-top-left-radius: 6px; \
				 border-bottom-left-radius: 6px;";
		break;
	case 1:
		break;
	case 2:
		style+= "border-top-right-radius: 6px; \
				 border-bottom-right-radius: 6px;";
		break;
	}

	style+= "} \
			QPushButton:pressed { \
				background-color: #d7392e; \
			} \
			QPushButton:checked { \
				background-color: #d7392e; \
			} \
			QPushButton:flat { \
				border: none; \
			}";
	setStyleSheet(style.c_str());
	setCheckable(true);
	setMinimumSize(80, 50);
}

void ControlButton::mouseReleaseEvent(QMouseEvent *e)
{
	QPushButton::mouseReleaseEvent(e);
	if (!isDown())
		setChecked(true);
	parent->buttonClicked(index);

}
