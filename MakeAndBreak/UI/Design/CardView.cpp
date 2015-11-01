/*
 * CardView.cpp
 *
 *  Created on: Jun 26, 2013
 *      Author: aida
 */

#include "CardView.h"
#include "DrawBlock.h"
#include "MakeAndBreak/UI/UIConstants.h"

#include <Box2D/Box2D.h>
#include <QTimerEvent>

InfoPanel::~InfoPanel()
{
	//TODO
}

InfoPanel::InfoPanel(QWidget *parent)
: QGraphicsView(parent)
{
	QGraphicsScene *sc = new QGraphicsScene();
	setScene(sc);
	resize(UI::INFO_WIDTH, UI::INFO_HEIGHT);
	UI::init(this, true);

	sc->setSceneRect(0, 0, UI::INFO_WIDTH, UI::INFO_HEIGHT);
	sc->setBackgroundBrush(QBrush(UI::DARK_BLUE));
	QLinearGradient bgl = QLinearGradient(UI::INFO_WIDTH / 2, 0, 0, 0);
	bgl.setColorAt(0, UI::BLUE);
	bgl.setColorAt(1, UI::DARK_BLUE);
	bgl.setSpread(QGradient::ReflectSpread);
	sc->setBackgroundBrush(bgl);

	layout = new QGridLayout();
	this->setLayout(layout);

	messages = new QLabel();
	messages->setStyleSheet("QLabel{ color : white; }");
	messages->setFont(UI::MAIN_FONT);
	layout->addWidget(messages, 0, 0, 1, 1, Qt::AlignVCenter | Qt::AlignLeft);

	cv = new CardView();
	cv->setRepeat(true);
	layout->addWidget(cv, 0, 2, 1, 1, Qt::AlignBottom | Qt::AlignRight);

	arrow = new QLabel();
	arrow->setPixmap(QPixmap("./data/makeAndBreak/Arrow.png"));
	layout->addWidget(arrow, 0, 1, 1, 1, Qt::AlignCenter);
	arrow->hide();
}

CardView::CardView(QWidget *parent)
: QGraphicsView(parent), timerId(0), world(NULL)
{
	QGraphicsScene *s = new QGraphicsScene();
	s->setItemIndexMethod(QGraphicsScene::NoIndex);
	s->setSceneRect(0, 0, UI::SIMULATOR_WIDTH, UI::SIMULATOR_HEIGHT);
	setFrameShadow(QFrame::Plain);
	setFrameShape(QFrame::Box);
	setStyleSheet("QGraphicsView { background: transparent; border: none; }");
	setScene(s);
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	reset();
	repeat = false;
}

void CardView::setBlocks(std::vector< std::pair<Block, QColor> > a)
{
	std::vector<Block> b;
	std::vector<QColor> c;
	for (int i = 0; i < (int) a.size(); i++) {
		b.push_back(a[i].first);
		c.push_back(a[i].second);
	}
	setBlocks(b, c);
}

void CardView::setBlocks(std::vector<Block> b, std::vector<QColor> c)
{
	blocks = b;
	colors = c;

	for (int i = 0; i < (int) dBlocks.size(); i++)
		dBlocks[i].remove();
	dBlocks.resize(0);

	for (int i = 0; i < (int) blocks.size(); i++)
		dBlocks.push_back(DrawBlock(
				blocks[i].x + 2 * 3,
				blocks[i].y,
				blocks[i].width,
				blocks[i].height,
				scene(), world, colors[i],
				UI::SIMULATOR_BLOCK_WIDTH,
				0,
				3 * 3 * UI::SIMULATOR_BLOCK_WIDTH + 5));
}

void CardView::reset()
{
	timerId = 0;
	restart();
}

void CardView::restart()
{
	b2Vec2 gravity = b2Vec2(0, -10);

	SAFE_DELETE(world);
	world = new b2World(gravity);

	//Ground
	ground.resize(0);
	for (int i = 0; i < 6; i++) {
		DrawBlock db((i * 3 + 1.5), (0 - 0.5), 3, 1, scene(), world, UI::BLACK,
				UI::SIMULATOR_BLOCK_WIDTH, 0, 3 * 3 * UI::SIMULATOR_BLOCK_WIDTH + 5);
		db.body->SetType(b2_staticBody);
		ground.push_back(db);
	}

	//Walls
	for (int i = 0; i < 3; i++) {
		DrawBlock db(0 - 0.5, (i * 3 + 1.5), 1, 3, scene(), world, UI::BLACK,
				UI::SIMULATOR_BLOCK_WIDTH, 0, 3 * 3 * UI::SIMULATOR_BLOCK_WIDTH + 5);
		db.hide = true;
		db.body->SetType(b2_staticBody);
		ground.push_back(db);

		db = DrawBlock(6 * 3 + 0.5, (i * 3 + 1.5), 1, 3, scene(), world, UI::BLACK,
				UI::SIMULATOR_BLOCK_WIDTH, 0, 3 * 3 * UI::SIMULATOR_BLOCK_WIDTH);
		db.hide = true;
		db.body->SetType(b2_staticBody);
		ground.push_back(db);
	}
	counter = 0;
}

void CardView::start()
{
	if (!timerId)
		timerId = startTimer(1000.0 / UI::STEPS_PER_SECOND);
}

void CardView::setRepeat(bool stat)
{
	repeat = stat;
}

CardView::~CardView()
{
	SAFE_DELETE(world);
}

void CardView::timerEvent(QTimerEvent *event)
{
	if (event->timerId() == timerId) {
		world->Step(1.0 / UI::STEPS_PER_SECOND, 8, 3);
		if (repeat && counter % (UI::STEPS_PER_SECOND * 2) == 0) {
			bool isStable = true;
			for (int i = 0; i < (int) dBlocks.size(); i++)
				if (dBlocks[i].body->IsAwake())
					isStable = false;
			if (isStable) {
				restart();
				setBlocks(blocks, colors);
			}
		}

		for (int i = 0; i < (int) dBlocks.size(); i++)
			dBlocks[i].display();
		for (int i = 0; i < (int) ground.size(); i++)
			ground[i].display();
		counter++;
	}
	QObject::timerEvent(event);
}
