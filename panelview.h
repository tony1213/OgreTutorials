#ifndef PANELWINDOW_H
#define PANELWINDOW_H
#include <QWidget>

#include <QColor>
#include <QSharedPointer>
#include <QTimer>
#include <QSlider>
#include <QLabel>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QString>
#include "mycustomslider.h"

class PanelView;
class Robot;
#define INDEX_WIDTH 50
#define VAR_WIDTH   100
#define SLIDER_WIDTH  150
#define LINE_MARGIN  10
#define LINE_HEIGHT  20


class PanelView : public QWidget
{
    Q_OBJECT

public:
    PanelView(QWidget* parent );
    ~PanelView();
    void update();
    void setRobot(Robot* robot);
protected:

    void paintEvent(QPaintEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);


private Q_SLOTS:
 
   void updateRobotAngle0(int value);
   void updateRobotAngle1(int value);
   void updateRobotAngle2(int value);
   void updateRobotAngle3(int value);
   void updateRobotAngle4(int value);
   void updateRobotAngle5(int value);
   void updateRobotAngle6(int value);
   void updateRobotAngle7(int value);
   void updateRobotAngle8(int value);
   void updateRobotAngle9(int value);
   void updateRobotAngle10(int value);
   void updateRobotAngle11(int value);
   void updateRobotAngle12(int value);
   void updateRobotAngle13(int value);
   void updateRobotAngle14(int value);
   void updateRobotAngle15(int value);

   //modify parameter...
   void modifyParameterVar0(int value);
   void modifyParameterVar1(int value);
   void modifyParameterVar2(int value);
   void modifyParameterVar3(int value);
   void modifyParameterVar4(int value);
   void modifyParameterVar5(int value);
   void modifyParameterVar6(int value);
   void modifyParameterVar7(int value);
   void modifyParameterVar8(int value);
   void modifyParameterVar9(int value);
   void modifyParameterVar10(int value);
   void modifyParameterVar11(int value);
   void modifyParameterVar12(int value);
   void modifyParameterVar13(int value);
   void modifyParameterVar14(int value);
   void modifyParameterVar15(int value);

private:
   Robot* robot_;
   QWidget* mParent; 

   QLabel *idLabel;
   QLabel *varLabel;
   QLabel *angleLabel;

   QLabel *label[16];
   QSpinBox *var[16];
   MyCustomSlider *slider[16];

   QString linkNames[21];

   void initVariablesUI();
   void UpdateRobot(const std::string& linkname, int valuH);

};
#endif
