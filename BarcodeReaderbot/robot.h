#ifndef ROBOT_H
#define ROBOT_H
#define PINUM 3.141559265

#include <QObject>
#include <QtDebug>
#include <QDebug>
#include <QtMath>
#include <QTimer>
#include "dynamixel.h"
#include "scanner.h"
#include "math.h"
#include "tcpserver.h"
#include "datamodel.h"
class Robot : public QObject
{
    Q_OBJECT

    Dynamixel _AX12;
    DataModel _dataModel;
    QTimer updateRobot;
    //bool lastState=false;
    //int forwardSpeed=1034;
    //int TargetX=100;
    //int TargetY=100;
    Scanner _scanner;
    int forwardSpeed;
    double preError;
    double integral;
    Point lastPos;
public:
    explicit Robot(QObject *parent = 0);

    void Stop();
    void GoBack(int speedR, int speedL);
    void GoForward(int speedR, int speedL);
    ~Robot();
    Line getLineFunc(Point Start, Point End);
    double GetDistance(Line line, Point point);
    double PIDControl(int setpoint, double actualValue, int Dt, double Kp, double Ki, double Kd, int max, int min);
    Point GetCurrentPosition();
    void PIDInit();
    double GetDistancePoint(Point start, Point end);
    double findAngle(Point start,Point end);
    void turnRobot(double finalAngle);
    void GotoPoint(Point goal);
    void GotoPosition(Point goal, double finalAngle);
    void checkHeading(Point end);
    int whichSide(Point A, Point B, Point goal);
signals:

public slots:
    void Cleanup();
};

#endif // ROBOT_H
