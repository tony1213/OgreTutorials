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
#include <tinyxml.h>
#include <urdf_model/model.h>

#define RANGE  10000


PanelView::PanelView(QWidget* parent)
 :QWidget(parent),
 idLabel(NULL),
 varLabel(NULL),
 angleLabel(NULL)
{
    robot_ = NULL; 
    initVariablesUI();

    needSend = false; 
    jointstate_pub = _privateHandle.advertise<sensor_msgs::JointState>("joint_states", 10);
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
    needSend = false; 
}


void PanelView::setRobot(Robot* robot){

    robot_ = robot;
    initJointStates();

    /*
    pid_t pid = fork();
    if(pid == 0){
        qDebug(">>>>start loop topic sending...");
        ROS_ERROR(">>>>start loop topic sending>>>chenrui");
        needSend = true;
        loopSending();
    }else
        qDebug(">>>>start loop topic sending failed...");

     */  
    pthread_t tid;
    pthread_create(&tid,NULL,work_thread,this);
  

}

void* PanelView::work_thread(void* arg)
{
    PanelView* curP = (PanelView *)arg;  
    curP->needSend = true;
    curP->loopSending();

    return 0;
}

/** init JointStates for all joints*/
void PanelView::initJointStates(){

    if(NULL == robot_){
        qDebug("error! robot_ is NULL");
    }
    urdf::Model urdf = robot_->getUrdfModel();

    typedef std::map<std::string, urdf::JointSharedPtr > M_NameToUrdfJoint;
    M_NameToUrdfJoint::const_iterator joint_it = urdf.joints_.begin();
    M_NameToUrdfJoint::const_iterator joint_end = urdf.joints_.end();
    for( ; joint_it != joint_end; ++joint_it )
    {
      const urdf::JointConstSharedPtr&  urdf_joint = joint_it->second;


      if(urdf_joint->type == urdf::Joint::FLOATING || urdf_joint->type == urdf::Joint::FIXED)
          continue;

      const urdf::JointLimitsConstSharedPtr& limits = urdf_joint->limits;

      Joint joint;
      joint.effort   = limits->effort;
      joint.lower    = limits->lower;
      joint.upper    = limits->upper;
      joint.velocity = limits->velocity;

      joint.max  = limits->upper;
      joint.min  =  limits->lower;
      joint.position = 0;
      
      if(joint.min > 0 || joint.max < 0)
          joint.zero = (joint.max + joint.min)/2; 
      else
          joint.zero = 0; 

      joint_list.push_back(urdf_joint->name);
      free_joints[urdf_joint->name] = joint; 

    }
     
}

/**loop to send Joinstates to tf system*/
void PanelView::sendJointStatesToTf(){
   
         
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    int  number = free_joints.size();
    int factor = 1; 
    int offset = 0; 


    for(int i = 0; i< number; i++){
        std::string  name = joint_list.at(i);
        Joint joint = free_joints[name];
        msg.name.push_back(name);
        msg.position.push_back(joint.position * factor + offset) ; 
        msg.velocity.push_back(joint.velocity * factor);
        msg.effort.push_back(joint.effort) ;  
    }
   
    ROS_ERROR(">>>>>sendJointStatesToTf>>>chenrui"); 
    jointstate_pub.publish(msg);

}


void PanelView::loopSending(){
    ros::Duration two_seconds(2); 
    while (ros::ok() && needSend )
    {
        sendJointStatesToTf();
        two_seconds.sleep();
    } 


}
void PanelView::UpdateRobot(const std::string& jointname, int valuH){
    Joint joint = free_joints[jointname];
    float pctvalue = valuH / float(RANGE); 
   // joint['min'] + (joint['max']-joint['min']) * pctvalue
    double radiant = joint.min + (joint.max - joint.min) * pctvalue;  
    free_joints[jointname].position = radiant ; 

   // sendJointStatesToTf();
   
   // robot_->updateRobot(jointname, valuH); 


}




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
        var[i]->setRange(0, RANGE);  
        slider[i]->setRange(0, RANGE);
        slider[i]->setValue(RANGE/2);  
                 

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



        //init jointNames which reprensent every robot revolute jonit
        jointNames[0]= "LShoulderRoll_joint";
        jointNames[1]= "LShoulderPitch_joint"; //LShoulderPitch
        jointNames[2]= "LElbow_joint";
        jointNames[3]= "LHipRoll_joint";
        jointNames[4]= "LHipPitch_joint";
        jointNames[5]= "LKnee_joint";
        jointNames[6]= "LAnklePitch_joint";
        jointNames[7]= "LAnkleRoll_joint";

        jointNames[8]= "RShoulderRoll_joint";
        jointNames[9]= "RShoulderPitch_joint";
        jointNames[10]= "RElbow_joint";
        jointNames[11]= "RHipRoll_joint";
        jointNames[12]= "RHipPitch_joint";
        jointNames[13]= "RKnee_joint";
        jointNames[14]= "RAnklePitch_joint";
        jointNames[15]= "RAnkleRoll_joint";

}


void PanelView::updateRobotAngle0(int value){

    UpdateRobot(jointNames[0], value);

}

void PanelView::updateRobotAngle1(int value){
    UpdateRobot(jointNames[1], value);
}
void PanelView::updateRobotAngle2(int value){

    UpdateRobot(jointNames[2], value);

}
void PanelView::updateRobotAngle3(int value){

    UpdateRobot(jointNames[3], value);  
}
void PanelView::updateRobotAngle4(int value){

    UpdateRobot(jointNames[4], value);
}
void PanelView::updateRobotAngle5(int value){

    UpdateRobot(jointNames[5], value);

}
void PanelView::updateRobotAngle6(int value){

    UpdateRobot(jointNames[6], value);
}
void PanelView::updateRobotAngle7(int value){
    UpdateRobot(jointNames[7], value);

}


void PanelView::updateRobotAngle8(int value){
    UpdateRobot(jointNames[8], value);

}
void PanelView::updateRobotAngle9(int value){
   UpdateRobot(jointNames[9], value);

}
void PanelView::updateRobotAngle10(int value){
    UpdateRobot(jointNames[10], value);

}
void PanelView::updateRobotAngle11(int value){
    UpdateRobot(jointNames[11], value);

}
void PanelView::updateRobotAngle12(int value){
    UpdateRobot(jointNames[12], value);

}
void PanelView::updateRobotAngle13(int value){
    UpdateRobot(jointNames[13], value);

}

void PanelView::updateRobotAngle14(int value){
    UpdateRobot(jointNames[14], value);

}


void PanelView::updateRobotAngle15(int value){

    UpdateRobot(jointNames[15], value);
}



//*********************************************************following is to change according to var 


void PanelView::modifyParameterVar0( int value){


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


