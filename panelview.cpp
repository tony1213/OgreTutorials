#include "panelview.h"
#include <QDebug>
#include <QSlider>
#include <QLabel>
#include <QPainter>

#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QString>
#include <sstream>
#include "robot.h"

PanelView::PanelView(QWidget* parent)
 :QWidget(parent),
 idLabel(NULL),
 varLabel(NULL),
 angleLabel(NULL)
{
    robot_ = NULL; 
    initVariablesUI();
}

PanelView::~PanelView()
{
    delete idLabel; 
    idLabel = NULL; 
    delete varLabel; 
    varLabel = NULL; 
    delete angleLabel; 
    angleLabel = NULL; 

    for(int i = 0; i< 16; i++){
        delete label[i];
        delete var[i]; 
        delete slider[i]; 

        label[i] = NULL;
        var[i] = NULL; 
        slider[i] = NULL; 
    }
}


void PanelView::setRobot(Robot* robot){

    robot_ = robot; 

}
/*
string PanelView::num2str(int i){
    stringstream ss;
    ss<<i;
    return ss.str();
}
*/
/**init panel board UI*/
void PanelView::initVariablesUI(){

    QPalette pe;
    pe.setColor(QPalette::WindowText,Qt::white);
    QFont lbl_font;
    lbl_font.setPointSize(14);

    //the description line 
    idLabel = new QLabel(this);
    varLabel = new QLabel(this);
    angleLabel = new QLabel(this);

    idLabel->setGeometry(QRect(200, 30, 50, 30));
    idLabel->setText("ID");
    idLabel->setPalette(pe);
    idLabel->setFont(lbl_font);
    idLabel->setAlignment(Qt::AlignCenter);

    
    varLabel->setGeometry(QRect(250, 30, 100, 30));
    varLabel->setText("修正量");
    varLabel->setPalette(pe);
    varLabel->setFont(lbl_font);
    varLabel->setAlignment(Qt::AlignCenter);
    
    angleLabel->setGeometry(QRect(350, 30, 150, 30));
    angleLabel->setText("角度");
    angleLabel->setPalette(pe);
    angleLabel->setFont(lbl_font);
    angleLabel->setAlignment(Qt::AlignCenter);

    int height = 30;
    QString number;  
    for(int i = 0; i< 16; i++){
        height = height + LINE_HEIGHT + LINE_MARGIN;
        
        label[i] = new QLabel(this);
        label[i]->setGeometry(QRect(200, height, 50, 20));
        number = QString::number(i+1,10);
        label[i]->setText(number);
        label[i]->setPalette(pe);
        label[i]->setFont(lbl_font);


        var[i] = new QSpinBox(this);
        var[i]->setGeometry(QRect(250, height, 100, 20));

        slider[i] = new MyCustomSlider(this);
        slider[i]->setOrientation(Qt::Horizontal);
        slider[i]->setGeometry(QRect(360, height, 150, 20));

        //here, perhaps different var has different range value
        var[i]->setRange(0, 130);  
        slider[i]->setRange(0, 130);  
                 

    }

        QObject::connect(slider[0], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle0(int))); 
        QObject::connect(slider[1], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle1(int)));
        QObject::connect(slider[2], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle2(int)));
        QObject::connect(slider[3], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle3(int)));
        QObject::connect(slider[4], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle4(int)));
        QObject::connect(slider[5], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle5(int)));
        QObject::connect(slider[6], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle6(int)));
        QObject::connect(slider[7], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle7(int)));
        QObject::connect(slider[8], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle8(int)));
        QObject::connect(slider[9], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle9(int)));
        QObject::connect(slider[10], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle10(int)));
        QObject::connect(slider[11], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle11(int)));
        QObject::connect(slider[12], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle12(int)));
        QObject::connect(slider[13], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle13(int)));
        QObject::connect(slider[14], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle14(int)));
        QObject::connect(slider[15], SIGNAL(valueChanged(int)),
                    this, SLOT(updateRobotAngle15(int)));

        QObject::connect(var[0], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar0(int)));
        QObject::connect(var[1], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar1(int)));
        QObject::connect(var[2], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar2(int)));
        QObject::connect(var[3], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar3(int)));
        QObject::connect(var[4], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar4(int)));
        QObject::connect(var[5], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar5(int)));
        QObject::connect(var[6], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar6(int)));
        QObject::connect(var[7], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar7(int)));
        QObject::connect(var[8], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar8(int)));
        QObject::connect(var[9], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar9(int)));
        QObject::connect(var[10], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar10(int)));
        QObject::connect(var[11], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar11(int)));
        QObject::connect(var[12], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar12(int)));
        QObject::connect(var[13], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar13(int)));
        QObject::connect(var[14], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar14(int)));
        QObject::connect(var[15], SIGNAL(valueChanged(int)),
                    this, SLOT(modifyParameterVar15(int)));



        //init linkNames which reprensent every robot link
        linkNames[0]= "LShoulderRoll_joint";
        linkNames[1]= "LShoulderPitch_joint"; //LShoulderPitch
        linkNames[2]= "LElbow_joint";
        linkNames[3]= "LHipRoll_joint";
        linkNames[4]= "LHipPitch_joint";
        linkNames[5]= "LKnee_joint";
        linkNames[6]= "LAnklePitch_joint";
        linkNames[7]= "LAnkleRoll_joint";

        linkNames[8]= "RShoulderRoll_joint";
        linkNames[9]= "RShoulderPitch_joint";
        linkNames[10]= "RElbow_joint";
        linkNames[11]= "RHipRoll_joint";
        linkNames[12]= "RHipPitch_joint";
        linkNames[13]= "RKnee_joint";
        linkNames[14]= "RAnklePitch_joint";
        linkNames[15]= "RAnkleRoll_joint";

}


void PanelView::updateRobotAngle0(int value){

    UpdateRobot("LShoulderRoll_joint", value);

}

void PanelView::updateRobotAngle1(int value){

    UpdateRobot("LShoulderPitch_joint", value);

}
void PanelView::updateRobotAngle2(int value){

    UpdateRobot("LElbow_joint", value);

}
void PanelView::updateRobotAngle3(int value){

    UpdateRobot("LHipRoll_joint", value);

}
void PanelView::updateRobotAngle4(int value){

    UpdateRobot("LHipPitch_joint", value);

}
void PanelView::updateRobotAngle5(int value){

    UpdateRobot("LKnee_joint", value);

}
void PanelView::updateRobotAngle6(int value){

    UpdateRobot("LAnklePitch_joint", value);

}
void PanelView::updateRobotAngle7(int value){

    UpdateRobot("LAnkleRoll_joint", value);

}

/*
        linkNames[8]= "RShoulderRoll_joint";
        linkNames[9]= "RShoulderPitch_joint";
        linkNames[10]= "RElbow_joint";
        linkNames[11]= "RHipRoll_joint";
        linkNames[12]= "RHipPitch_joint";
        linkNames[13]= "RKnee_joint";
        linkNames[14]= "RAnklePitch_joint";
        linkNames[15]= "RAnkleRoll_joint";

*/

void PanelView::updateRobotAngle8(int value){

    UpdateRobot("RShoulderRoll_joint", value);

}
void PanelView::updateRobotAngle9(int value){

    UpdateRobot("RShoulderPitch_joint", value);

}
void PanelView::updateRobotAngle10(int value){

    UpdateRobot("RElbow_joint", value);

}
void PanelView::updateRobotAngle11(int value){

    UpdateRobot("RHipRoll_joint", value);

}
void PanelView::updateRobotAngle12(int value){

    UpdateRobot("RHipPitch_joint", value);

}
void PanelView::updateRobotAngle13(int value){

    UpdateRobot("RKnee_joint", value);

}

void PanelView::updateRobotAngle14(int value){

    UpdateRobot("RAnklePitch_joint", value);

}


void PanelView::updateRobotAngle15(int value){

    UpdateRobot("RAnkleRoll_joint", value);

}


void PanelView::modifyParameterVar0( int value){

    UpdateRobot("RKnee", value);

}

void PanelView::modifyParameterVar1( int value){


}
void PanelView::modifyParameterVar2( int value){


}
void PanelView::modifyParameterVar3( int value){


}
void PanelView::modifyParameterVar4( int value){


}
void PanelView::modifyParameterVar5( int value){


}
void PanelView::modifyParameterVar6( int value){


}
void PanelView::modifyParameterVar7( int value){


}
void PanelView::modifyParameterVar8( int value){


}
void PanelView::modifyParameterVar9( int value){


}
void PanelView::modifyParameterVar10( int value){


}
void PanelView::modifyParameterVar11( int value){


}
void PanelView::modifyParameterVar12( int value){


}
void PanelView::modifyParameterVar13( int value){


}

void PanelView::modifyParameterVar14( int value){


}


void PanelView::modifyParameterVar15( int value){


}



void PanelView::UpdateRobot(const std::string& linkname, int valuH){

    if(NULL != robot_){

        robot_->updateRobot(linkname, valuH);

    }

}
void PanelView::update(){

    QPainter p(this);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::black);
    p.drawRect(rect());



}
void PanelView::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
   

    QPainter p(this);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::black);
    p.drawRect(rect());
    
}

void PanelView::mousePressEvent(QMouseEvent* event)
{
}

void PanelView::mouseMoveEvent(QMouseEvent *event){
}


