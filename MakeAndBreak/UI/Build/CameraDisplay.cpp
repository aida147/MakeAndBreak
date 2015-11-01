/*
 * CameraDisplay.cpp
 *
 *  Created on: Jul 26, 2013
 *      Author: aida
 */

#include "CameraDisplay.h"

//#include <mab_msgs/MakeAndBreakCameraInfo.h>

CameraDisplay::CameraDisplay(QWidget *parent)
: QGraphicsView(parent)
{
	QGraphicsScene *s = new QGraphicsScene();
	s->setSceneRect(0, 0, UI::CAMERA_WIDTH, UI::CAMERA_HEIGHT);
	setScene(s);
	setFrameShadow(QFrame::Plain);
	setFrameShape(QFrame::Box);
	setStyleSheet("QGraphicsView { background : transparent; border: none; }");
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	numOfImages = 2;
	sourceNames.resize(numOfImages);
	sourceNames[KINECT_RGB] = "Color Camera";
	sourceNames[OCS_VIEW] = "3D Perception";

	sourceButtons.resize(0);

	init();
}

CameraDisplay::~CameraDisplay() {
	for (int i = 0; i < (int) sourceButtons.size(); i++)
		SAFE_DELETE(sourceButtons[i]);
}

void CameraDisplay::switchSource(int i)
{
	if(i == KINECT_RGB)
	{
	    imageDisplay->setImageName("Kinect/RGB");
	}
	else if(i == OCS_VIEW)
	{
	    imageDisplay->setImageName("Kinect/DepthGray");
	}
}

void CameraDisplay::init()
{
	QGridLayout *layout = new QGridLayout();
	setLayout(layout);

	sourceButtons.resize(numOfImages);
	for (int i = 0; i < (int) sourceButtons.size(); i++) {
		sourceButtons[i] = new SourceButton(i, this);
		sourceButtons[i]->setMinimumHeight(50);
		sourceButtons[i]->setText(sourceNames[i].c_str());
		layout->addWidget(sourceButtons[i], 0, i, 1, 1);
	}

	imageDisplay = new QtImageCacheDisplay();
	imageDisplay->resize(UI::CAMERA_WIDTH, UI::CAMERA_HEIGHT);
	layout->addWidget(imageDisplay, 1, 0, 1, -1, Qt::AlignCenter);
}
