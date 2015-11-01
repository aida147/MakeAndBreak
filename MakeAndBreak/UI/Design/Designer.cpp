/*
 * Designer.cpp
 *
 *  Created on: Jun 27, 2013
 *      Author: aida
 */

#include "Designer.h"
#include "MakeAndBreak/UI/UIConstants.h"
#include "MakeAndBreak/Math.h"

#include <iostream>
#include <cmath>
#include <string>

#include <Box2D/Box2D.h>
#include <QTimerEvent>
#include <QGraphicsRectItem>
#include <QObject>
#include <QGraphicsProxyWidget>
#include <QColor>

#include "Utilities/Debug.h"

Designer::~Designer() {
	SAFE_DELETE(buildButton);
	SAFE_DELETE(label);
	SAFE_DELETE(instructions);
	SAFE_DELETE(logo);
}

Designer::Designer(MainWindow *mw, QWidget *parent)
: QGraphicsView(parent), timerId(0)
{
	layout = new QGridLayout();
	QGraphicsScene *scene = new QGraphicsScene();
	scene->setBackgroundBrush(QBrush(UI::DARK_BLUE));
	this->setScene(scene);
	layout->setRowMinimumHeight(5, 20);
	layout->setRowStretch(0, 1);
	layout->setRowStretch(7, 1);
	layout->setColumnStretch(0, 1);
	layout->setColumnStretch(9, 1);

	//BUILDING AREA
	mainWindow = mw;
	buildingArea = new BuildingArea(this);
	layout->addWidget(buildingArea, 1, 5, 4, 1);

	//LOGO
	logo = new QLabel();
	logo->setPixmap(QPixmap("./data/makeAndBreak/RI_logo.png"));
	layout->addWidget(logo, 6, 1, 1, 1, Qt::AlignCenter);

	//INSTRUCTIONS
	initInstructions();

	//BUTTON
	initButton();

	//INFO
	info = new InfoPanel();
	layout->addWidget(info, 6, 3, 1, 5);
	updateCV();

	this->setLayout(layout);
}

void Designer::initInstructions()
{
	std::string s = "\n\n\n";
	s+= "Step 1: Design a pattern.\n    -Drag blocks into the grid.\n    -Double tap on a block to\n     rotate it.";
	s+= "\n\n\n";
	s+= "Step 2: Tap on GO! button on\n    the right to tell the robot to\n    start building your pattern.";
	s+= "\n\n\n";
	s+= "*Read the messages below. If\nyour pattern can't be built,\nthey will tell you why.";

	instructions = new QLabel();
	layout->addWidget(instructions, 2, 1, 3, 3, Qt::AlignLeft | Qt::AlignTop);
	instructions->setStyleSheet("QLabel { color : white; }");
	instructions->setText(s.c_str());
	instructions->setFont(UI::MAIN_FONT);

	label = new QLabel();
	layout->addWidget(label, 1, 1, 1, 3, Qt::AlignLeft | Qt::AlignBottom);
	label->setStyleSheet("QLabel { color : white; }");
	label->setText("Instructions:");
	label->setFont(UI::BOLD_FONT);
}

void Designer::initButton()
{
	buildButton = new QPushButton();
	buildButton->resize(UI::BUTTON_WIDTH, UI::BUTTON_HEIGHT);
	layout->addWidget(buildButton, 3, 7, 1, 2, Qt::AlignCenter);
	connect(buildButton, SIGNAL(released()), this, SLOT(handleButton()));
	buildButton->setIcon(QIcon(QPixmap("./data/makeAndBreak/button.png")));
	buildButton->setIconSize(QSize(UI::BUTTON_WIDTH, UI::BUTTON_HEIGHT));
	buildButton->setFlat(true);
	QPalette palette = buildButton->palette();
	palette.setColor(QPalette::Button, Qt::transparent);
	palette.setColor(QPalette::Base, Qt::transparent);
	palette.setColor(QPalette::Background, Qt::transparent);
	buildButton->setPalette(palette);
}

void Designer::updateCV()
{
	info->cv->reset();
	info->cv->setBlocks(getPattern());
	info->cv->start();
}

void Designer::addMessage(QString message, bool showArrow)
{
	clearMessage();
	info->messages->setText(message);
	messageTimer = 0;
	if (showArrow)
		info->arrow->show();
}

void Designer::clearMessage()
{
	info->messages->setText("");
	info->arrow->hide();
}

void Designer::handleButton()
{
	std::vector< std::pair<Block, QColor> > t = getPattern();
	std::vector<Block> b;
	for (int i = 0; i < (int) t.size(); i++)
		b.push_back(t[i].first);
	Solver s(b);
	s.useBothHandsWithTwoHandActions();
	if (b.size() < 1) {
		addMessage("Put at least one block in the grid.");
		return;
	}
	if (!s.isStable()) {
		addMessage("Please make a stable pattern.", true);
		return;
	}

	mainWindow->showWaiting();
	buildingArea->setActive(false);
	buildButton->setEnabled(false);

	mainWindow->moves = s.solve();
	if (mainWindow->moves.size() == 0) {
		addMessage("Robot can't build this.");
		mainWindow->hideWaiting();
		buildingArea->setActive(true);
		buildButton->setEnabled(true);
		//Probably requires putting more than 2 blocks at the same time
		return;
	}

	//TODO
	std::vector<Move> a = mainWindow->moves;
	int counter = 0;
	for (int i = 0; i < (int) a.size(); i++) {
		if (a[i].left != Move::NO_BLOCK) {
			counter++;
			std::cerr << "left ";
		}
		if (a[i].right != Move::NO_BLOCK) {
			counter--;
			std::cerr << "right ";
		}
	}
	std::cerr << counter << "-------------------\n";

	mainWindow->blocks = b;
	std::vector<QColor> c;
	for (int i = 0; i < (int) t.size(); i++)
		c.push_back(t[i].second);
	mainWindow->colors = c;
	convert();

	mainWindow->switchState();
}

void Designer::convert()
{
	std::vector<Move> solution = mainWindow->moves;
	std::vector<Block> b = mainWindow->blocks;

	mainWindow->serviceData.resize(0);

	if (solution.size() > 0 && solution[0].mode == Move::ONE_HAND &&
			b[solution[0].getBlock()].height > b[solution[0].getBlock()].width)
		if ((b[solution[0].getBlock()].left() < 3 && solution[0].left != Move::NO_BLOCK)
				|| (b[solution[0].getBlock()].left() >= 3 && solution[0].right != Move::NO_BLOCK))
			solution[0].changeHands();

	for (int i = 0; i < (int) solution.size(); i++) {
		blockMessage block[2];
		for (int h = 0; h < 2; h++) {
			int c = h == 0 ? solution[i].left : solution[i].right;
			if (c == Move::NO_BLOCK) {
				block[h].isValid = false;
			} else {
				int relto = -1;
				double angle = M_PI / 2;

				int visiblity = -1;
				float top = -1;
				for (int j = 0; j < i; j++) {
					for (int h = 0; h < 2; h++) {
						int c = h == 0 ? solution[j].left : solution[j].right;
						if (c == Move::NO_BLOCK)
							continue;
						if (FM::greater(b[c].top(), top)) {
							top = b[c].top();
							relto = c;
							visiblity = b[c].width > b[c].height ?
									3 : 1;
						} else if (FM::equ(b[c].top(), top) &&
								b[c].width > b[c].height &&
								visiblity < 3) {
							top = b[c].top();
							relto = c;
							visiblity = 3;
						}
					}
				}

				std::pair<double, double> pos(b[c].x, b[c].y);
				if (relto > -1) {
					pos.first -= b[relto].xCenter();
					pos.second -= b[relto].yCenter();
				}

				for (int k = 0; k < 2 * i; k++) {
					int j = (k % 2 == 0) ? solution[k/2].left : solution[k/2].right;
					if (j == Move::NO_BLOCK || j == c)
						continue;

					if (h == 0 && b[c].leftTo(b[j])) {
						angle = 3 * M_PI / 4;
					}
					if (h == 1 && b[c].rightTo(b[j])) {
						angle = M_PI / 4;
					}
				}
				block[h].orientation = b[c].width < b[c].height;
				block[h].position = pos;
				block[h].relativeTo = relto;
				block[h].id = c;
				block[h].angle = angle;
				block[h].isValid = true;
				block[h].colorR = mainWindow->colors[c].red();
				block[h].colorG = mainWindow->colors[c].green();
				block[h].colorB = mainWindow->colors[c].blue();

				std::cerr << block[h].orientation << " " <<
						block[h].position.first << "," <<
						block[h].position.second << " " <<
						block[h].relativeTo << " " <<
						block[h].id << " " <<
						block[h].angle << " " <<
						block[h].isValid << "\n";
			}
		}
		mainWindow->serviceData.push_back(std::pair<blockMessage, blockMessage>(block[0], block[1]));
	}
}

void Designer::start()
{
	if (!timerId)
		timerId = startTimer(1000.0 / UI::STEPS_PER_SECOND);
	stopped = false;
}

void Designer::stop()
{
	stopped = true;
}

void Designer::timerEvent(QTimerEvent *event)
{
	if (!stopped && event->timerId() == timerId) {
		for (int i = 0; i < (int) buildingArea->blocks.size(); i++)
			if (buildingArea->blocks[i]->isCurrentError) {
				addMessage(buildingArea->blocks[i]->error);
				buildingArea->blocks[i]->isCurrentError = false;
			}
		if (messageTimer == UI::STEPS_PER_SECOND * 20)
			clearMessage();
		messageTimer++;
	}
	QObject::timerEvent(event);
}

std::vector< std::pair<Block, QColor> > Designer::getPattern()
{
	return buildingArea->getPattern();
}
