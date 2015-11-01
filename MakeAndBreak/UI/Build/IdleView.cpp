/*
 * IdleView.cpp
 *
 *  Created on: Jul 8, 2013
 *      Author: aida
 */

#include "IdleView.h"
#include "MakeAndBreak/UI/UIConstants.h"
#include "RosClients/MakeAndBreakDataClient.h"

#include <iostream>

#include <QPushButton>
#include <QTimerEvent>
#include <QLinearGradient>
#include <QRadialGradient>

IdleView::~IdleView()
{
	SAFE_DELETE(m_nodeHandle);
	SAFE_DELETE(timer);
	SAFE_DELETE(commands);
	SAFE_DELETE(camera);
}

IdleView::IdleView(MainWindow *mw, QWidget *parent)
: QGraphicsView(parent), timerId(0), mainWindow(mw), timer(NULL),
  commands(NULL), camera(NULL)
{
	scene = new QGraphicsScene();
	scene->setSceneRect(0, 0, UI::WIDTH, UI::HEIGHT);
	scene->setBackgroundBrush(QBrush(UI::DARK_BLUE));
	setScene(scene);

	layout = new QGridLayout();
	layout->setRowStretch(1, 1);
	layout->setRowStretch(0, 2);
	layout->setRowStretch(3, 2);
	layout->setVerticalSpacing(0);
	layout->setRowMinimumHeight(4, 20);
	setLayout(layout);

	m_nodeHandle = new ros::NodeHandle();
	m_subState = m_nodeHandle->subscribe("MakeAndBreakOcsState", 1000, &IdleView::setAppState, this);
	m_subBehavior = m_nodeHandle->subscribe("MakeAndBreakOcsBehavior", 1000, &IdleView::updateCommands, this);
	ocs_state = MAKE_BREAK_OCS_STATE_BUILD;
	counter = 0;

	m_subStep = m_nodeHandle->subscribe("MakeAndBreakStepNumber", 1000, &IdleView::updateStepNumber, this);

	init();
}

void IdleView::initCommands()
{
	QGraphicsView *commandsWidget = new QGraphicsView();
	QGraphicsScene *sc = new QGraphicsScene();
	commandsWidget->setScene(sc);
	commandsWidget->setFrameShadow(QFrame::Plain);
	commandsWidget->setFrameShape(QFrame::Box);
	commandsWidget->setStyleSheet(UI::WIDGET_STYLE);
	commandsWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	commandsWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	commandsWidget->setMinimumSize(UI::COMMANDS_WIDTH, UI::COMMANDS_HEIGHT);

	QGridLayout *l = new QGridLayout();
	commandsWidget->setLayout(l);

	QLinearGradient bgr = QLinearGradient(UI::COMMANDS_WIDTH / 2, 0, 0, 0);
	bgr.setColorAt(1, UI::MID_DARK_BLUE);
	bgr.setColorAt(0, UI::DARK_BLUE);
	bgr.setSpread(QGradient::ReflectSpread);
	sc->setBackgroundBrush(QBrush(bgr));

	commands = new QLabel();
	commands->setFont(UI::MAIN_FONT);
	commands->setStyleSheet("color : white");
	l->addWidget(commands, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignTop);

	layout->addWidget(commandsWidget, 1, 0, 2, 1);
}

void IdleView::initCameraAndTimer()
{
	QGraphicsView *tv = new QGraphicsView();
	QGraphicsScene *s = new QGraphicsScene();
	s->setSceneRect(0, 0, UI::COMMANDS_WIDTH, UI::COMMANDS_HEIGHT);
	tv->setScene(s);
	tv->setFrameShadow(QFrame::Plain);
	tv->setFrameShape(QFrame::Box);
	tv->setStyleSheet(UI::WIDGET_STYLE);
	tv->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	tv->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	tv->setMinimumSize(UI::COMMANDS_WIDTH, UI::COMMANDS_HEIGHT);

	QLinearGradient bgr = QLinearGradient(UI::COMMANDS_WIDTH / 2, 0, 0, 0);
	bgr.setColorAt(0, UI::MID_DARK_BLUE);
	bgr.setColorAt(1, UI::DARK_BLUE);
	bgr.setSpread(QGradient::ReflectSpread);
	s->setBackgroundBrush(QBrush(bgr));

	QGridLayout *l = new QGridLayout();
	tv->setLayout(l);
	l->setRowStretch(0, 1);

	timer = new QLabel(tv);
	QFontMetrics fm(UI::BOLD_FONT);
	timer->setFont(UI::BOLD_FONT);
	timer->setStyleSheet("QLabel { color : yellow; }");
	timer->setText(toMinutesAndSeconds(0));
	timer->resize(UI::CAMERA_WIDTH, UI::TIMER_HEIGHT);
	timer->setAlignment(Qt::AlignCenter);
	l->addWidget(timer, 0, 0, 1, 1);

	camera = new CameraDisplay();
	camera->resize(UI::CAMERA_WIDTH, UI::CAMERA_HEIGHT);
	l->addWidget(camera, 1, 0, 1, 1);

	layout->addWidget(tv, 1, 4, 2, 1);
}

void IdleView::initSteps()
{
	QGraphicsView *stepsWidget = new QGraphicsView();
	layout->addWidget(stepsWidget, 5, 0, 1, 5);
	QGraphicsScene *sc = new QGraphicsScene();
	sc->setBackgroundBrush(QBrush(UI::DARK_BLUE));
	sc->setSceneRect(0, 0, UI::STEP_WIDTH, UI::STEP_HEIGHT);
	stepsWidget->setScene(sc);
	stepsWidget->setFrameShadow(QFrame::Plain);
	stepsWidget->setFrameShape(QFrame::Box);
	stepsWidget->setStyleSheet(UI::WIDGET_STYLE);
	stepsWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	stepsWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	QLinearGradient bgl = QLinearGradient(UI::STEP_WIDTH / 2, 0, 0, 0);
	bgl.setColorAt(0, UI::BLUE);
	bgl.setColorAt(1, UI::DARK_BLUE);
	bgl.setSpread(QGradient::ReflectSpread);
	sc->setBackgroundBrush(bgl);

	steps.clear();
	std::vector<Block> bt;
	std::vector<QColor> ct;

	for (int i = 0; i < (int) mainWindow->moves.size(); i++) {
		float x = mainWindow->moves.size() * UI::STEP_BOX_WIDTH + (mainWindow->moves.size() - 1) * UI::H_SPACE;
		x/= -2;
		x+= UI::WIDTH / 2;
		x+= i * (UI::STEP_BOX_WIDTH + UI::H_SPACE);
		float y = UI::V_SPACE;

		if (mainWindow->moves[i].right != Move::NO_BLOCK) {
			bt.push_back(mainWindow->blocks[mainWindow->moves[i].right]);
			ct.push_back(mainWindow->colors[mainWindow->moves[i].right]);
		}
		if (mainWindow->moves[i].left != Move::NO_BLOCK) {
			bt.push_back(mainWindow->blocks[mainWindow->moves[i].left]);
			ct.push_back(mainWindow->colors[mainWindow->moves[i].left]);
		}

		steps.push_back(Step(bt, ct, sc, x, y));

		if (i+1 == (int) mainWindow->moves.size())
			continue;

		QGraphicsPixmapItem *t = new QGraphicsPixmapItem(QPixmap("./data/makeAndBreak/Next.png"));
		//t->scale(0.1, 0.1);
		t->setPos(x + UI::STEP_BOX_WIDTH + UI::H_SPACE / 2 - 10,
				UI::V_SPACE + UI::STEP_BOX_HEIGHT / 2 - 7);
		sc->addItem(t);

	}
}

void IdleView::initRobot()
{
	robotModel = new RobotView();
	layout->addWidget(robotModel, 0, 2, 4, 1);
	robotModel->resize(UI::MODEL_WIDTH, UI::MODEL_HEIGHT);
}

void IdleView::init()
{
	initCommands();
	initCameraAndTimer();
	initSteps();
	initRobot();
}

std::string sub(std::string main, std::string s)
{
	int ind = main.size() - 4;
	for (int i = 0; i < 3; i++)
		main[ind + i] = s[i];
	return main;
}

QString IdleView::toMinutesAndSeconds(int time)
{
	std::string res = "00:00";
	//			       01234
	int t = time / UI::STEPS_PER_SECOND;
	res[0] = '0' + (t / 600) % 6;
	res[1] = '0' + (t / 60) % 10;
	res[3] = '0' + (t / 10) % 6;
	res[4] = '0' + t % 10;
	return QString(res.c_str());
}

void IdleView::setAppState(const mab_msgs::MakeAndBreakOcsState& msg)
{
	ARMS_INFO("M&B OCS State : " << msg.state);
	ocs_state = static_cast<MakeAndBreakOcsState>(msg.state);
}

void IdleView::updateStepNumber(const mab_msgs::MakeAndBreakStepNumber& msg)
{
	ARMS_INFO("M&B Step : " << msg.step);
	if (msg.step < (int) mainWindow->moves.size()) {
		if (msg.step > 0)
			steps[msg.step - 1].setState(Step::PAST);
		steps[msg.step].setState(Step::CURRENT);
	} else {
		steps[(int) mainWindow->moves.size() - 1].setState(Step::PAST);
	}
}

void IdleView::updateCommands(const mab_msgs::MakeAndBreakOcsBehavior& msg)
{
	ARMS_INFO("M&B OCS Behavior : " << msg.name);
	addCommand(msg.name);
}

void IdleView::addCommand(std::string s)
{
	std::string t = commands->text().toStdString() +
			"[" + toMinutesAndSeconds(counter).toStdString() + "]  " + s + "\n";
	int counter = 0;
	for (int i = 0; i < (int) t.size(); i++)
		if (t[i] == '\n')
			counter++;
	for (int i = 0; i < counter - 17; i++)
		t = t.substr(t.find('\n') + 1, t.size() - t.find('\n') - 1);
	commands->setText(t.c_str());
}

void IdleView::handleButton()
{
	mainWindow->switchState();
}

void IdleView::start()
{
	if (!timerId)
		timerId = startTimer(1000.0 / UI::STEPS_PER_SECOND);
}

void IdleView::timerEvent(QTimerEvent *event)
{
	if (event->timerId() == timerId) {
		if (ocs_state == MAKE_BREAK_OCS_STATE_IDLE
				&& mainWindow->getState() == MAKE_BREAK_APP_STATE_IDLE)
			handleButton();
		timer->setText(toMinutesAndSeconds(counter));
		counter++;
	}
	QObject::timerEvent(event);
}
