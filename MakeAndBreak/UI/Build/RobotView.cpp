/*
 * RobotView.cpp
 *
 *  Created on: Jul 31, 2013
 *      Author: mklingen
 */

#include "RobotView.h"
#include "Environment/EnvironmentRenderer.h"
#include "Environment/EnvironmentRenderer.h"
#include "Environment/Cameras/ArcBallCamera.h"
#include "Environment/ObjectTypes/MeshObject.h"
#include "eigen3/Eigen/src/Geometry/Transform.h"
#include <QVBoxLayout>
#include <QTimer>



RobotView::RobotView(QColor bg, QWidget* parent, Qt::WindowFlags f) :
    QWidget(parent, f), m_backgroundColor(bg)
{
    QString fonttemplate = tr("<font color='%1'>%2</font>");

    arms::Environment& environment = arms::Environments::get("live");
    arms::EnvironmentRenderer renderer = arms::EnvironmentRenderer(environment, new arms::ArcBallCamera(true));
    renderer.selectionColor = Eigen::Vector4d(1, 1, 0, 1);

    QWidgetList list = QApplication::topLevelWidgets();

    int maxWidth = 0;
    int maxHeight = 0;

    for(int i = 0; i < list.count(); i++)
    {
        maxWidth = std::max(maxWidth, list.at(i)->size().width());
        maxHeight = std::max(maxHeight, list.at(i)->size().height());
    }

    QGridLayout *layout = new QGridLayout();
    setLayout(layout);

    QFont font = UI::MAIN_FONT;
    font.setPointSize(12);

    m_envRenderer = new QtEnvironmentRendererWidget(renderer, NULL, parent,0,0,false);
    m_envRenderer->show();
    layout->addWidget(m_envRenderer, 0, 0, 1, 2);

    if (UI::MakeAndBreakIdleViewMode == UI::OLD_IDLE_VIEW) {
    	m_jointValuesLeft = new QLabel(this);
    	m_jointValuesLeft->setFont(font);
    	m_jointValuesLeft->setText( fonttemplate.arg( "white", "Left Joints" ) );
    	layout->addWidget(m_jointValuesLeft, 1, 0, 1, 1);

    	m_selectedLabel = new QLabel(this);
    	m_selectedLabel->setFont(font);
    	m_selectedLabel->setText( fonttemplate.arg( "white", "" ) );
    	layout->addWidget(m_selectedLabel, 1, 1, 1, 1);

    	m_jointValuesRight = new QLabel(this);
    	m_jointValuesRight->setFont(font);
    	m_jointValuesRight->setText( fonttemplate.arg( "white", "Right Joints" ) );
    	layout->addWidget(m_jointValuesRight, 2, 0, 1, 1);
    }

    createFakeZoomEvent();

    updateMeshColors();
    jointValueUpdate();

    QTimer* timer = new QTimer(this);
    timer->setInterval(500);
    timer->setSingleShot(false);
    timer->start();
    connect(timer, SIGNAL(timeout()), this, SLOT(jointValueUpdate()));
}

RobotView::~RobotView()
{

}

void RobotView::createFakeZoomEvent()
{
    QWheelEvent *event =
            new QWheelEvent(QPoint(UI::MODEL_WIDTH / 2, UI::MODEL_HEIGHT / 2),
            QPoint(UI::WIDTH / 2, UI::HEIGHT / 2),
            600,
            Qt::MouseButtons(),
            Qt::KeyboardModifiers());
    QCoreApplication::sendEvent(m_envRenderer, event);
}

void RobotView::setWhiteWireframeRecursive(arms::Object* object)
{
    Eigen::Vector4d meshColor(UI::LIGHT_GRAY.redF(),
            UI::LIGHT_GRAY.greenF(),
            UI::LIGHT_GRAY.blueF(),
            UI::LIGHT_GRAY.alphaF());
    arms::MeshObject* mesh = dynamic_cast<arms::MeshObject*>(object);

    if(mesh) {
        mesh->drawMode = arms::MeshResource::Wireframe;
        mesh->renderPreference = arms::MeshObject::GeometryMesh;
        mesh->rigid = false;
        mesh->color = meshColor;
    } else {
        object->visible = false;
    }

    for(size_t i =0; i < object->parts.size(); i++)
        setWhiteWireframeRecursive(object->parts[i]);
}

void RobotView::updateMeshColors()
{
    std::vector<arms::Object*> meshes = arms::Environments::live().getObjects();
    for(size_t i = 0; i < meshes.size(); i++)
    {
        setWhiteWireframeRecursive(meshes[i]);
    }

    QColor bg = m_backgroundColor;
    arms::Environments::live().backgroundColor = Eigen::Vector4d(
            bg.redF(), bg.greenF(), bg.blueF(), bg.alphaF());

}

void RobotView::jointValueUpdate()
{
    std::string ds = " ";
    ds[0] = (char) 176;
    QString fonttemplate = tr("<font color='%1'>%2</font>");
    arms::Vector7d jointsRight;
    arms::Vector7d jointsLeft;

    arms::Environments::live().getWam(arms::RIGHT_ARM)->getJointValues(jointsRight);
    arms::Environments::live().getWam(arms::LEFT_ARM)->getJointValues(jointsLeft);

    std::stringstream rightLabel;
    rightLabel << "Right Joints:";

    std::stringstream leftLabel;
    leftLabel << "Left Joints:";

    for(int i = 0; i < 7; i++)
    {
        rightLabel << (int)arms::RadToDeg(jointsRight(i)) << ds;
        leftLabel << (int)arms::RadToDeg(jointsLeft(i)) << ds;

        if(i < 6)
        {
            rightLabel << ",";
            leftLabel << ",";
        }
    }



    if (UI::MakeAndBreakIdleViewMode == UI::OLD_IDLE_VIEW) {
    	m_jointValuesLeft->setText( fonttemplate.arg( "white", rightLabel.str().c_str() ) );
    	m_jointValuesRight->setText( fonttemplate.arg( "white", leftLabel.str().c_str() ) );

    	if(m_envRenderer->renderer.selectionUid >= 0)
    	{
    		arms::Object* selected = arms::Environments::live().find(m_envRenderer->renderer.selectionUid);

    		if(selected)
    		{
    			std::string name = selected->name;
    			for (int i = 0; i < (int) name.size(); i++)
    				if (name[i] == '_')
    					name[i] = ' ';
    			m_selectedLabel->setText( fonttemplate.arg( "white", ("Selected: " + name).c_str() ) );
    		}
    }
    }
}


