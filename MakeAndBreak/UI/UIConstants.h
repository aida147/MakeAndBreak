/*
 * UIConstants.h
 *
 *  Created on: Jul 12, 2013
 *      Author: aida
 */

#ifndef UICONSTANTS_H_
#define UICONSTANTS_H_

#include <QColor>
#include <QFont>
#include <QGraphicsView>
#include <vector>

#define SAFE_DELETE(x) { if( (x) ) { delete (x); (x) = NULL; }}

namespace UI {
	extern float WIDTH,
			HEIGHT,
			MAX_NUM_OF_BLOCKS,
			MAIN_BLOCK_WIDTH,
			MAIN_BLOCK_LENGTH,
			SIMULATOR_BLOCK_WIDTH,
			STEPS_BLOCK_WIDTH;

	extern float LOGO_WIDTH;

	extern float INFO_WIDTH,
			INFO_HEIGHT;

	extern float BUILDING_WIDTH,
			BUILDING_HEIGHT;

	extern float GRID_LEFT,
			GRID_RIGHT,
			GRID_TOP,
			GRID_BOTTOM;

	extern float SIMULATOR_WIDTH,
			SIMULATOR_HEIGHT;

	extern float BUTTON_WIDTH,
			BUTTON_HEIGHT;

	extern float STEP_BOX_WIDTH,
			STEP_BOX_HEIGHT,
			STEP_WIDTH,
			STEP_HEIGHT,
			H_SPACE,
			V_SPACE;

	extern float COMMANDS_WIDTH,
			COMMANDS_HEIGHT;

	extern float TIMER_HEIGHT,
			CAMERA_WIDTH,
			CAMERA_HEIGHT;

	extern float CAMERA_CONTROL_WIDTH,
			CAMERA_CONTROL_LENGTH;

	extern float MODEL_WIDTH,
			MODEL_HEIGHT;

	extern int GRID_COLUMNS, GRID_ROWS;
	extern int BLOCK_LENGTH, BLOCK_WIDTH;

	extern int numOfColors;
	extern QColor list[];
	extern QColor LIGHT_GRAY, BLACK, YELLOW;
	extern QColor LIGHT_BLUE, BLUE, DARK_BLUE, MID_DARK_BLUE, LIGHT_GREEN;

	extern QFont MAIN_FONT, BOLD_FONT, OBLIQUE_FONT, BOLD_OBLIQUE_FONT;

	extern int STEPS_PER_SECOND;

	extern QString	WIDGET_BORDER, WIDGET_STYLE;

	enum{
		OLD_IDLE_VIEW = 0,
		NEW_IDLE_VIEW = 1
	};

	extern int MakeAndBreakIdleViewMode;

	void init();
	void init(QGraphicsView *view, bool hasBorder = false);
};


#endif /* UICONSTANTS_H_ */
