#ifndef CONTROLLER_H
#define CONTROLLER_H

#define PINUM 3.141559265
#define DISAREA 25
#define CHECKANGLE 1
#define DIS_TO_GOAL 10
#define READ_ERROR_THRESHOLD 1
#define CARROT_OFFSET 40

#define BORDER_Y_MIN 10
#define BORDER_Y_MAX 10
#define BORDER_X_MIN 10
#define BORDER_X_MAX 10

#include <QtDebug>
#include <QtMath>
#include <QTimer>
#include <QDateTime>
#include <QQueue>
#include <QMutex>
#include "dynamixel.h"
#include "scanner.h"
#include "controller.h"
#include "math.h"
#include "tcpserver.h"
#include "datamodel.h"
#include "xl320.h"


#include <QObject>
#include <QDebug>
#include <QThread>
#include "scanner.h"
#include "xl320.h"
#include "tcpserver.h"


class Controller : public QObject
{
    Q_OBJECT
public:

    XL320 *_AX12;
    Point currentPos;
    double currentAngle;
    Point lastPos;
    double lastAngle;
    Scanner _scanner;
    int forwardSpeed;
    double preError;
    double integral;
    bool mission_done;
    bool mission_stop;
    bool forward;
    QQueue<Point> Goals;
    Point _goal;
    Point carrot;
    double _kp;
    double _kd;
    double _ki;
    double angle_offset;
    QTimer updateTimer;
    QTimer *controllerTimer;
    QTimer *serverDataTimer;
    QMutex dataMutex;
    Controller();
    ~Controller();


    Point currentPosPID;
    Line linePID;
    Point StartPID;
    double finalAngle;
    double currentDisPID;
    int maxPID;
    int motorOffsetPID;
    int minPID;
    int readErrorCounterPID;

    void Stop();
    Line getLineFunc(Point Start, Point End);
    int whichSide(Point A, Point B, Point goal);
    double GetDistancePoint(Point start, Point end);
    double GetDistance(Line line, Point point);
    void PIDInit();
    double PIDControl(int setpoint, double actualValue, int Dt, double Kp, double Ki, double Kd, int max, int min);
    Point GetCurrentPosition();
    void checkHeading(Point end);
    double findAngle(Point start, Point end);
    void turnRobot(double finalAngle);
    void turnRight(unsigned long val);
    void turnLeft(unsigned long val);
    void Init(QQueue<Point> goal, double kp, double kd, double ki);
    bool findCurrentAngle(int numberOfTry);


    void PrintPoint(QString msg, Point point);
    void GoForward(int speedR, int speedL, int offsetR = 0, int offsetL = 0);
    void GoBack(int speedR, int speedL, int offsetR = 0, int offsetL = 0);
    void getCarrotPosition(Point present, Line line, Point start, Point goal);
    bool findAngleOffset();
    double SensorAngle();
public slots:
    void UpdateStatus();
    void Start();
    void StopRobot();
    void startPID();
    void sendServerData();
signals:
    void Dummy();
    void UpdatePoint(int,int,double,int,int);
    void WorkFinished();

};

#endif // CONTROLLER_H
