#include "panelview.h"
#include <QDebug>
#include <QSlider>
#include <QLabel>
#include <QPainter>

#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QString>
#include <sstream>

PanelView::PanelView(QWidget* parent)
 :QWidget(parent),
 idLabel(NULL),
 varLabel(NULL),
 angleLabel(NULL)
{
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

    for(int i = 0; i< 14; i++){
        delete label[i];
        delete var[i]; 
        delete slider[i]; 

        label[i] = NULL;
        var[i] = NULL; 
        slider[i] = NULL; 
    }
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
    for(int i = 0; i< 14; i++){
        height = height + LINE_HEIGHT + LINE_MARGIN;
        
        label[i] = new QLabel(this);
        label[i]->setGeometry(QRect(200, height, 50, 20));
        number = QString::number(i+1,10);
        label[i]->setText(number);
        label[i]->setPalette(pe);
        label[i]->setFont(lbl_font);


        var[i] = new QSpinBox(this);
        var[i]->setGeometry(QRect(250, height, 100, 20));

       // slider[i] = new QSlider(this);
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

        //init linkNames which reprensent every robot link
        linkNames[0]= "Torso";
        linkNames[1]= "Neck";
        linkNames[2]= "Head";
        linkNames[3]= "LShoulderPitch";
        linkNames[4]= "LShoulderRoll";
        linkNames[5]= "LElbow";
        linkNames[6]= "LHipYaw";
        linkNames[7]= "LHipRoll";

        linkNames[8]= "LHipPitch";
        linkNames[9]= "LKnee";
        linkNames[10]= "LAnklePitch";
        linkNames[11]= "LAnkleRoll";
        linkNames[12]= "RShoulderPitch";
        linkNames[13]= "RShoulderRoll";
        linkNames[14]= "RElbow";
        linkNames[15]= "RHipYaw";
        linkNames[16]= "RHipRoll";
        linkNames[17]= "RHipPitch";
        linkNames[18]= "RKnee";
        linkNames[19]= "RAnklePitch";
        linkNames[20]= "RAnkleRoll";

}


void PanelView::updateRobotAngle0(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle0");

}

void PanelView::updateRobotAngle1(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle1");

}
void PanelView::updateRobotAngle2(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle2");

}
void PanelView::updateRobotAngle3(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle3");

}
void PanelView::updateRobotAngle4(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle4");

}
void PanelView::updateRobotAngle5(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle5");

}
void PanelView::updateRobotAngle6(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle6");

}
void PanelView::updateRobotAngle7(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle7");

}
void PanelView::updateRobotAngle8(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle8");

}
void PanelView::updateRobotAngle9(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle9");

}
void PanelView::updateRobotAngle10(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle10");

}
void PanelView::updateRobotAngle11(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle11");

}
void PanelView::updateRobotAngle12(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle12");

}
void PanelView::updateRobotAngle13(int value){

    qDebug(">>>>>chenrui>>>>updateRobotAngle13");

}


void PanelView::modifyParameterVar0( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar0");

}

void PanelView::modifyParameterVar1( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar1");

}
void PanelView::modifyParameterVar2( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar2");

}
void PanelView::modifyParameterVar3( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar3");

}
void PanelView::modifyParameterVar4( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar4");

}
void PanelView::modifyParameterVar5( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar5");

}
void PanelView::modifyParameterVar6( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar6");

}
void PanelView::modifyParameterVar7( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar7");

}
void PanelView::modifyParameterVar8( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar8");

}
void PanelView::modifyParameterVar9( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar9");

}
void PanelView::modifyParameterVar10( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar10");

}
void PanelView::modifyParameterVar11( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar11");

}
void PanelView::modifyParameterVar12( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar12");

}
void PanelView::modifyParameterVar13( int value){

    qDebug(">>>>>chenrui>>>modifyParameterVar13");

}


void PanelView::UpdateRobot(QString linkname, int valuH){







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


