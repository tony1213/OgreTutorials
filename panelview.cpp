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

        slider[i] = new QSlider(this);
        slider[i]->setOrientation(Qt::Horizontal);
        slider[i]->setGeometry(QRect(360, height, 150, 20));

        //here, perhaps different var has different range value
        var[i]->setRange(0, 130);  
        slider[i]->setRange(0, 130);  
        //
                 
        QObject::connect(var[i], SIGNAL(valueChanged(int)),
                    slider[i], SLOT(setValue(int)));  
        QObject::connect(slider[i], SIGNAL(valueChanged(int)),
                    var[i], SLOT(setValue(int))); 


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
   
    qDebug(">>>>>>>PanelView::paintEvent>>>>");

    QPainter p(this);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::black);
    p.drawRect(rect());
    
}

void PanelView::mousePressEvent(QMouseEvent* event)
{
    qDebug("PanelView::mousePressEvent");
}

void PanelView::mouseMoveEvent(QMouseEvent *event){
    qDebug()<<"mouseMoveEvent:";
}


