/*
 * RobotView.h
 *
 *  Created on: Jul 31, 2013
 *      Author: mklingen
 */

#ifndef ROBOTVIEW_H_
#define ROBOTVIEW_H_

#include <QtArmWidgets/QtEnvironmentRendererWidget.h>
#include "MakeAndBreak/UI/UIConstants.h"
#include <QWidget>
#include <QLabel>

class RobotView : public QWidget
{
        Q_OBJECT

    public:
        RobotView(QColor bg = UI::DARK_BLUE, QWidget* parent = NULL, Qt::WindowFlags f = 0);
        virtual ~RobotView();

        QtEnvironmentRendererWidget* m_envRenderer;
        QLabel* m_jointValuesLeft;
        QLabel* m_jointValuesRight;
        QLabel* m_selectedLabel;
        QColor m_backgroundColor;


        void updateMeshColors();
        void createFakeZoomEvent();
        void setWhiteWireframeRecursive(arms::Object* object);

    public slots:
        void jointValueUpdate();

};


#endif /* ROBOTVIEW_H_ */
