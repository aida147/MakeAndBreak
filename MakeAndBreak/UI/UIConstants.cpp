/*
 * UIConstants.cpp
 *
 *  Created on: Jul 12, 2013
 *      Author: aida
 */

#include "UIConstants.h"

namespace UI {
	float   WIDTH = 1200,
			HEIGHT = 675,
			MAX_NUM_OF_BLOCKS = 7,
			MAIN_BLOCK_WIDTH = 40,
			MAIN_BLOCK_LENGTH = 120,
			SIMULATOR_BLOCK_WIDTH = 13,
			STEPS_BLOCK_WIDTH = 17;

	float	LOGO_WIDTH = 80;

	float	INFO_WIDTH = 1080 - 210,
			INFO_HEIGHT = 140;

	float	SIMULATOR_WIDTH = SIMULATOR_BLOCK_WIDTH * 3 * 6,
			SIMULATOR_HEIGHT = SIMULATOR_BLOCK_WIDTH * 3 * 3.35;

	float	BUTTON_WIDTH = 210,
			BUTTON_HEIGHT = 90;

	int     GRID_COLUMNS = 6, GRID_ROWS = 9;
	int     BLOCK_LENGTH = 3, BLOCK_WIDTH = 1;

	float	BUILDING_WIDTH = 620,
			BUILDING_HEIGHT = 520;

	float 	GRID_LEFT = (int) (BUILDING_WIDTH/2 - 2 * MAIN_BLOCK_WIDTH),
			GRID_RIGHT = (int) (GRID_LEFT + GRID_COLUMNS * MAIN_BLOCK_WIDTH),
			GRID_TOP = (int) (BUILDING_HEIGHT/2 - 2 * MAIN_BLOCK_WIDTH),
			GRID_BOTTOM = (int) (GRID_TOP + 9 * MAIN_BLOCK_WIDTH);

	float	STEP_BOX_WIDTH = 102,
			STEP_BOX_HEIGHT = 153,
			STEP_WIDTH = 1200,
			STEP_HEIGHT = 640 - 460,
			H_SPACE = 55,
			V_SPACE = 13.5;

	float	COMMANDS_WIDTH = 280 + 50,
			COMMANDS_HEIGHT = 320 + 37.5;

	float	TIMER_HEIGHT = 110,
			CAMERA_WIDTH = 640 / 2,
			CAMERA_HEIGHT = 480 / 2;

	float	CAMERA_CONTROL_WIDTH = 30,
			CAMERA_CONTROL_LENGTH = 1.54 * CAMERA_CONTROL_WIDTH;

	float	MODEL_WIDTH = 450,
			MODEL_HEIGHT = 520;


	int     numOfColors = 9;
	QColor  list[] = {
				QColor(252, 81, 32),	//red
				QColor(76, 170 , 10),	//light green
				QColor(31, 110, 151),	//blue
				QColor(242, 203, 7),	//yellow
				QColor(8, 103 , 30),	//green
				QColor(254, 98 , 41),	//orange
				QColor(137, 120 , 110),	//gray
				QColor(151, 66, 24),	//brown
				QColor(229, 220 , 214)	//white
	};
	QColor  LIGHT_GRAY = QColor(232,232,232), BLACK = QColor(51, 51, 51),
			YELLOW = QColor(242, 203, 7);

	QColor	LIGHT_BLUE = QColor(0, 151, 245), BLUE = QColor(17, 51, 131),
			DARK_BLUE = QColor(7, 18, 38), MID_DARK_BLUE = QColor(13, 39, 97),
			LIGHT_GREEN = QColor(76, 170 , 10);

	QFont	MAIN_FONT = QFont("Univers LT Std", 16, QFont::Light),
			BOLD_FONT = QFont("Univers LT Std", 18, QFont::Bold),
			OBLIQUE_FONT = QFont("Univers LT Std", 16, QFont::Light),
			BOLD_OBLIQUE_FONT = QFont("Univers LT Std", 18, QFont::Bold);

	QString	WIDGET_BORDER =
			"border-bottom : 1px solid rgb(0, 151, 245);\
			border-top : 1px solid rgb(0, 151, 245);\
			border-left : none;\
			border-right : none;\
			border-radius : 0;";
	QString WIDGET_STYLE = "QGraphicsView { background: transparent; " +
			WIDGET_BORDER + "}";

	int STEPS_PER_SECOND = 60;

	int MakeAndBreakIdleViewMode = OLD_IDLE_VIEW;

	void init()
	{
		UI::OBLIQUE_FONT.setStyle(QFont::StyleOblique);
		UI::BOLD_OBLIQUE_FONT.setStyle(QFont::StyleOblique);
	}

	void init(QGraphicsView *view, bool hasBorder)
	{
		if (hasBorder)
			view->setStyleSheet(WIDGET_STYLE);
		else
			view->setStyleSheet("QGraphicsView { background: transparent; border: none; }");
		view->setFrameShadow(QFrame::Plain);
		view->setFrameShape(QFrame::Box);
		view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	}
};
