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

class PanelView;

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
protected:

    void paintEvent(QPaintEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

private:
   QWidget* mParent; 

   QLabel *idLabel;
   QLabel *varLabel;
   QLabel *angleLabel;

   QLabel *label[14];
   QSpinBox *var[14];
   QSlider *slider[14];

   void initVariablesUI();
  // QString num2str(int i);

};
#endif
