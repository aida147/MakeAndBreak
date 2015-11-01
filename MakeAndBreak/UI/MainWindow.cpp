/*
 * MainWindow.cpp
 *
 *  Created on: Jul 1, 2013
 *      Author: aida
 */

#include "MainWindow.h"
#include "mab_msgs/MakeAndBreakAppState.h"
#include "UIConstants.h"

#include <fstream>
#include <cmath>

#include "Environment/EnvironmentRenderer.h"
#include "Environment/Cameras/ArcBallCamera.h"
#include "Environment/ObjectTypes/MeshObject.h"
#include "eigen3/Eigen/src/Geometry/Transform.h"

MainWindow::MainWindow(ros::Publisher* publisher) :
main(NULL), designer(NULL), idleView(NULL), builder(NULL), m_statePub(publisher)
{
	UI::init();
	readConf();
}

MainWindow::~MainWindow()
{
	SAFE_DELETE(main);
	SAFE_DELETE(designer);
	SAFE_DELETE(idleView);
}

void MainWindow::readConf()
{
	std::string line;
	std::ifstream conf("./data/makeAndBreak/conf");
	if (conf.is_open()) {
		while (conf.good()) {
			getline(conf, line);
			if (line.substr(0, 5) == "mouse") {
				if (line.substr(6, 4) == "hide")
					QApplication::setOverrideCursor(QCursor(Qt::BlankCursor));
			}
			if (line.substr(0, 8) == "idleview") {
				if (line.substr(9, 3) == "new")
					UI::MakeAndBreakIdleViewMode = UI::NEW_IDLE_VIEW;
				else
					UI::MakeAndBreakIdleViewMode = UI::OLD_IDLE_VIEW;
			}
		}
		conf.close();
	}
}

void MainWindow::init()
{
	main = new QWidget();
	main->resize(UI::WIDTH, UI::HEIGHT);
	main->setWindowFlags(Qt::FramelessWindowHint);
	main->showFullScreen();
	layout = new QBoxLayout(QBoxLayout::TopToBottom);
	main->setLayout(layout);

	setState(MAKE_BREAK_APP_STATE_DESIGN);
}

void MainWindow::publishState()
{
	mab_msgs::MakeAndBreakAppState msg;
	msg.state = static_cast<int>(m_state);
	m_statePub->publish(msg);
	ros::spinOnce();
}

void MainWindow::mouseReleased()
{
	if (m_state == MAKE_BREAK_APP_STATE_DESIGN) {
		designer->updateCV();
		// designer->clearMessage(); TODO
	}
}

void MainWindow::mousePressed()
{
	if (m_state == MAKE_BREAK_APP_STATE_DESIGN) {
		designer->clearMessage();
	}
}

MakeAndBreakAppState MainWindow::getState()
{
	return m_state;
}

void MainWindow::showWaiting()
{
	waiting = new QGraphicsView(main);
	QGraphicsScene *ts = new QGraphicsScene();
	ts->setBackgroundBrush(QBrush(UI::DARK_BLUE));
	ts->setSceneRect(0, 0, 550, 200);

	QFontMetrics fm = QFontMetrics(UI::BOLD_OBLIQUE_FONT);
	QGraphicsSimpleTextItem *msg = new QGraphicsSimpleTextItem();
	msg->setText("PLEASE WAIT");
	msg->setFont(UI::BOLD_OBLIQUE_FONT);
	ts->addItem(msg);
	msg->setPos(550 / 2 - fm.width("PLEASE WAIT") / 2, 40);
	msg->setBrush(QBrush(UI::YELLOW));

	QFontMetrics fm2 = QFontMetrics(UI::OBLIQUE_FONT);
	QGraphicsSimpleTextItem *msg2 = new QGraphicsSimpleTextItem();
	msg2->setText("Andy is planning...");
	msg2->setFont(UI::OBLIQUE_FONT);
	ts->addItem(msg2);
	msg2->setPos(550 / 2 - fm2.width("Andy is planning...") / 2, 100);
	msg2->setBrush(QBrush(UI::LIGHT_BLUE));

	waiting->setScene(ts);
	waiting->resize(550, 200);
	waiting->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	waiting->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	waiting->show();

	waiting->setFrameShape(QFrame::StyledPanel);
	waiting->setFrameShadow(QFrame::Raised);
	waiting->move(400, 250);
	waiting->resize(550, 200);
	waiting->show();

	QCoreApplication::processEvents();
	QCoreApplication::processEvents();
}

void MainWindow::hideWaiting()
{
	SAFE_DELETE(waiting);
}

void cleanLayout(QLayout *layout)
{
	if (layout != NULL) {
		QLayoutItem* item;
		while ((item = layout->takeAt(0)) != 0) {
			delete item;
		}
	}
}

void MainWindow::setState(MakeAndBreakAppState state)
{
	m_state = state;

	if (state == MAKE_BREAK_APP_STATE_DESIGN) {
		SAFE_DELETE(designer);
		designer = new Designer(this);
		designer->start();

		cleanLayout(layout);
		main->layout()->addWidget(designer);

		isReady = false;
	}
	if (state == MAKE_BREAK_APP_STATE_IDLE) {
		if (UI::MakeAndBreakIdleViewMode == UI::OLD_IDLE_VIEW) {
			SAFE_DELETE(idleView);
			idleView = new IdleView(this);
			idleView->start();
			isReady = true;

			cleanLayout(layout);
			main->layout()->addWidget(idleView);
		} else if (UI::MakeAndBreakIdleViewMode == UI::NEW_IDLE_VIEW) {
			SAFE_DELETE(builder);
			builder = new Builder(this);
			builder->start();
			isReady = true;

			cleanLayout(layout);
			main->layout()->addWidget(builder);
		}

		hideWaiting();
	}

	QCoreApplication::processEvents();
	publishState();
}

void MainWindow::switchState()
{
	if (m_state == MAKE_BREAK_APP_STATE_DESIGN)
		setState(MAKE_BREAK_APP_STATE_IDLE);
	else if (m_state == MAKE_BREAK_APP_STATE_IDLE)
		setState(MAKE_BREAK_APP_STATE_DESIGN);
}
