/*
 * CameraDisplay.h
 *
 *  Created on: Jul 26, 2013
 *      Author: aida
 */

#ifndef CAMERADISPLAY_H_
#define CAMERADISPLAY_H_

#include "MakeAndBreak/UI/UIConstants.h"
#include "MakeAndBreak/UI/MakeAndBreakDefines.h"
#include "MakeAndBreak/UI/MainWindow.h"

#include <QGraphicsScene>
#include <QPushButton>
#include <QGraphicsPixmapItem>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <QtArmWidgets/QtImageCacheDisplay.h>

#include <vector>

class SourceButton;
class ControlButton;
class MainWindow;

class CameraDisplay : public QGraphicsView {
	Q_OBJECT

	float width, height;
	std::vector<SourceButton *> sourceButtons;
	std::vector<std::string> sourceNames;
	int numOfImages;
	QtImageCacheDisplay* imageDisplay;

public:
	CameraDisplay(QWidget *parent = 0);
	virtual ~CameraDisplay();
	void init();
	void switchSource(int i);
};

class SourceButton : public QPushButton {
	Q_OBJECT
public:
	int index;
	CameraDisplay *parent;

	SourceButton(int index, CameraDisplay *parent) :
		QPushButton(), index(index), parent(parent)
	{
		QString style =
				"QPushButton { \
					border: 2px solid rgba(255, 255, 255, 10); \
					border-radius: 6px; \
					background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, \
					stop: 0 #1643af, stop: 1 #103180); \
					min-width: 80px; \
					color: #e8e8e8 \
				} \
				QPushButton:pressed { \
					background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, \
						stop: 0 #103180, stop: 1 #1643af); \
				} \
				QPushButton:flat { \
					border: none; \
				} \
				QPushButton:default { \
					border-color: navy; \
				}";
		setStyleSheet(style);
	}

	void mouseReleaseEvent (QMouseEvent *e)
	{
		QPushButton::mouseReleaseEvent(e);
		parent->switchSource(index);
	}
};

#endif /* CAMERADISPLAY_H_ */
