#ifndef ROBOT_H
#define ROBOT_H
#define PINUM 3.141559265
#define DISAREA 30
#define CHECKANGLE 0
#define DIS_TO_GOAL 10
#define READ_ERROR_THRESHOLD 50

#define BORDER_Y_MIN 10
#define BORDER_Y_MAX 10
#define BORDER_X_MIN 10
#define BORDER_X_MAX 10

#include <QObject>
#include <QtDebug>
#include <QDebug>
#include <QtMath>
#include <QTimer>
#include <QDateTime>
#include <QQueue>
#include <QThread>
#include "dynamixel.h"
#include "scanner.h"
#include "controller.h"
#include "math.h"
#include "tcpserver.h"
#include "datamodel.h"
#include "xl320.h"
class Robot : public QObject
{
    Q_OBJECT

    //Dynamixel _AX12;
    //XL320 _AX12;

    QThread *controllerThread;
    DataModel _dataModel;
    QTimer updateRobotTimer;
    Point currentPos;
    double currentAngle;
    Point lastPos;
    double lastAngle;
    TCPServer *server;
    Controller *controller;
    //bool lastState=false;
    //int forwardSpeed=1034;
    //int TargetX=100;
    //int TargetY=100;
    //Scanner _scanner;
    int forwardSpeed;
    double preError;
    double integral;
    bool mission_done;
    bool mission_stop;
    double kp;
    double kd;
    double ki;
    bool forward;
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

    void checkHeading(Point end);
    int whichSide(Point A, Point B, Point goal);


    double dotProduct(Point AB, Point CD);
    double magnitude(Point AB);
    double crossProduct(Point AB, Point CD);
    void GotoPointCrossProduct(Point goal);
    void GoBackR(int speedR);
    void GoBackL(int speedL);
    void GoForwardL(int speedL);
    void GoForwardR(int speedR);
    void turnLeft(int val);
    void turnRight(unsigned long val);
    void findCurrentAngle();
    Point GetExactCurrentPosition();
signals:
    void GoPathSignal(QQueue<Point>);
    void GoSignal(Point,double);
    void GoCircle(Point,int);
    void Dummy();
    void StopRobot();
public slots:
    void Cleanup();
    void FollowPath(QQueue<Point>);
    void GotoPosition(Point goal, double finalAngle);
    void Circle(Point center, int radius);
    void UpdateFinalPosition(QByteArray);
    void SendPositionToServer(int x, int y, double angle,int x_carrot,int y_carrot);
};

#endif // ROBOT_H
