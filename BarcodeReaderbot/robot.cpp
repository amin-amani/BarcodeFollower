#include "robot.h"

Robot::Robot(QObject *parent) : QObject(parent)
{
    Stop();

    //GotoPosition(temp,80);
    forwardSpeed=17;
_AX12.Init("/dev/ttyUSB0",1000000);


Point temp;
temp.x = 40;
temp.y = 150;
//qDebug("Angle:" + QString::number(findAngle()).toLatin1());
//turnRobot(false);
GotoPosition(temp,80);
}

void Robot::GoForward(int speedR, int speedL)
{
    _AX12.SetPosition(3,100,speedR - 1024);
    _AX12.SetPosition(1,100,speedL);

}
void Robot::GoBack(int speedR, int speedL)
{
    _AX12.SetPosition(3,100,speedR);
    _AX12.SetPosition(1,100,speedL - 1024);

}
void Robot::Stop()
{
    _AX12.SetPosition(3,100,1024);
    _AX12.SetPosition(1,100,0);
    _AX12.SetTorqueEnable(3,0);
    _AX12.SetTorqueEnable(1,0);

}

Line Robot::getLineFunc(Point Start,Point End)
{
    Line result;
    if(End.x == Start.x)
    {
        result.b = 0;
        result.a = 1;
        result.c = (-1)*(End.x);
        return result;
    }
    if(End.y == Start.y)
    {
        result.b = 1;
        result.a = 0;
        result.c = (-1)*(End.y);
        return result;
    }
    result.a = (End.y - Start.y);
    result.a /= (End.x - Start.x);
    result.c = End.y - (result.a*End.x);
    result.b = -1;
    return result;
}

int Robot::whichSide(Point A,Point B,Point goal){
   double result = ((goal.x-A.x)*(B.y-A.y)-(goal.y-A.y)*(B.x-A.x));
   return (result>0?1:-1);
}
double Robot::GetDistancePoint(Point start,Point end){
    double result = 0;
    result = (end.x-start.x)*(end.x-start.x) + (end.y-start.y)*(end.y-start.y);
    return sqrt(result);
}
double Robot::GetDistance(Line line,Point point)
{
    double result = 0;
   /* int co = 1;
    if(line.a > -1 && line.a <0)co=-1;
    else if(line.a < -1)co=1;
    else if(line.a > 1)co=1;
    else if(line.a > 0 && line.a < 1)co=-1;*/
    result = line.a*point.x + line.b*point.y + line.c;
    result /= sqrt((line.a*line.a)+(line.b*line.b));
    return ((result<0)?(-1*result):result);
}
void Robot::PIDInit(){
    preError = 0;
    integral = 0;
}
double Robot::PIDControl(int setpoint,double actualValue,int Dt,double Kp,double Ki,double Kd,int max, int min)//about 10 us
{
double result=0;
double err;
double derivative;


err = setpoint - actualValue;
integral = integral + (err * Dt);
derivative = (err - preError) / Dt;
preError=err;
result= (Kp * err) + (Ki * integral) + (Kd * derivative);
if(integral>max)integral=max;
else if(integral<min)integral=min;
if(result>max)return max;
if(result<min)return min;
return result;
}

Point Robot::GetCurrentPosition(){
    Point result = _dataModel.GetPointFromString(_scanner.Scan());
    while(result.x == -1){
        Stop();
        QThread::usleep(2);
        result = _dataModel.GetPointFromString(_scanner.Scan());
    }
    if(result.x != -1)
        lastPos = result;

    return lastPos;
}

void Robot::GotoPoint(Point goal){
    PIDInit();
    Point start = GetCurrentPosition();
    Line line = getLineFunc(start,goal);
    Point currentPos = start;
    double currentDis = 0;
    int max = 50;
    int motorOffset = 50;
    int min = -50;
    double kp = 2;
    double kd = 0.1;
    double ki = 0.05;
    Stop();
    while(GetDistancePoint(goal,currentPos)  > 5){
        qDebug("Dis:"+QString::number(GetDistancePoint(goal,currentPos)).toLatin1());
        qDebug("Goal("+QString::number(goal.x).toLatin1()+","+QString::number(goal.y).toLatin1()+")");
        qDebug("Current("+QString::number(currentPos.x).toLatin1()+","+QString::number(currentPos.y).toLatin1()+")");
        currentPos = GetCurrentPosition();
        currentDis = GetDistance(line,currentPos);
        double pid = PIDControl(0,whichSide(start,goal,currentPos)*currentDis,1,kp,ki,kd,max,min);
        GoForward(motorOffset - pid ,motorOffset + pid);
        qDebug("pid: " + QString::number(pid).toLatin1());
    }
    Stop();
}

void Robot::GotoPosition(Point goal,double finalAngle)
{
    double radius = 30;
    Stop();QThread::sleep(2);
    Point mid_goal;
    if((finalAngle == 0) || (finalAngle == 360) || (finalAngle == 90) || (finalAngle == 180) ||(finalAngle == 270)){
        if((finalAngle == 0) || (finalAngle == 360)){

            mid_goal.x = goal.x - radius;
            mid_goal.y = goal.y;
        }
        else  if(finalAngle == 180){
            mid_goal.x = goal.x + radius;
            mid_goal.y = goal.y;
        }
        else if(finalAngle == 90){
            mid_goal.y = goal.y - radius;
            mid_goal.x = goal.x;
        }
        else if(finalAngle == 270){
            mid_goal.y = goal.y + radius;
            mid_goal.x = goal.x;
        }
    }
    else{
        double m = tan((finalAngle*PINUM)/180);
        qDebug("m= " + QString::number(m).toLatin1());
        double x_offset = radius;
        x_offset /= sqrt(1+m*m);

        if((finalAngle > 0 && finalAngle < 90) || (finalAngle > 270 && finalAngle < 360))
        {
                mid_goal.x = goal.x - x_offset;

        }
        else if((finalAngle > 90 && finalAngle < 270))
        {
                mid_goal.x = goal.x + x_offset;

        }

        double diff = (mid_goal.x - goal.x);
        qDebug("diff= " + QString::number(diff).toLatin1());
        diff *= m;

        qDebug("diff= " + QString::number(diff).toLatin1());
        diff += goal.y;
        qDebug("diff= " + QString::number(diff).toLatin1());
        mid_goal.y = (int)(diff);
        qDebug("mid.y= " + QString::number(mid_goal.y).toLatin1());
    }
    checkHeading(mid_goal);
    GotoPoint(mid_goal);
    checkHeading(goal);
    GotoPoint(goal);

}

void Robot::checkHeading(Point end){
    Point start;
    start = GetCurrentPosition();

    GoForward(forwardSpeed,forwardSpeed); QThread::sleep(1);
    Point temp_end = GetCurrentPosition();
    double currentAngle = findAngle(start,temp_end);
    double finalAngle =  findAngle(start,end);
    qDebug("final Angle:"+QString::number(finalAngle).toLatin1());
    if(abs(finalAngle - currentAngle) > 90)
    {
        while(abs(finalAngle - currentAngle) > 20){
        _AX12.SetPosition(3,100,17 - 1024);
        _AX12.SetPosition(1,100,17 - 1024);
        QThread::sleep(1);Stop(); end = GetCurrentPosition(); QThread::usleep(200000);
        currentAngle = findAngle(start,end);
        qDebug("current Angle: "+QString::number(currentAngle).toLatin1());
        }
    }
}
/*void Robot::GotoPosition(Point goal,double finalAngle)
{
    Stop();QThread::sleep(2);
    Point start;
    start.x = -1;
    while(start.x == -1)
        start = _dataModel.GetPointFromString(_scanner.Scan());
    Point mid_goal;
    double m = tan((finalAngle*3.141559265)/180);
    qDebug("m= " + QString::number(m).toLatin1());
    if((finalAngle == 0) || (finalAngle == 180) || (finalAngle == 360)){

        mid_goal.x = start.x;
        mid_goal.y = goal.y;
    }
    else if((finalAngle > 0 && finalAngle <= 45) || (finalAngle > 135 && finalAngle < 180) || (finalAngle > 180 && finalAngle <= 225) || (finalAngle > 315 && finalAngle < 360))
    {
            mid_goal.x = start.x;
            double diff = (mid_goal.x - goal.x);
            qDebug("diff= " + QString::number(diff).toLatin1());
            diff *= m;

            qDebug("diff= " + QString::number(diff).toLatin1());
            diff += goal.y;
            qDebug("diff= " + QString::number(diff).toLatin1());
            mid_goal.y = (int)(diff);
            qDebug("mid.y= " + QString::number(mid_goal.y).toLatin1());
    }
    else if((finalAngle > 45 && finalAngle < 90) || (finalAngle > 90 && finalAngle <= 135) || (finalAngle > 225 && finalAngle < 270) || (finalAngle > 270 && finalAngle <= 315))
    {
            mid_goal.y = start.y;
            double diff = (mid_goal.y - goal.y);
            qDebug("diff= " + QString::number(diff).toLatin1());
            diff /= m;

            qDebug("diff= " + QString::number(diff).toLatin1());
            diff += goal.x;
            qDebug("diff= " + QString::number(diff).toLatin1());
            mid_goal.x = (int)(diff);
            qDebug("mid.x= " + QString::number(mid_goal.x).toLatin1());
    }
    else if((finalAngle == 90) || (finalAngle == 270)){
        mid_goal.y = start.y;
        mid_goal.x = goal.x;
    }

    GotoPoint(mid_goal);
    GotoPoint(goal);

}*/
void Robot::turnRobot(double finalAngle){
    double currentAngle = 111000;
    forwardSpeed=17;
    Point start ;
    Point end ;
    _AX12.SetPosition(3,100,12 - 1024);
    _AX12.SetPosition(1,100,12);
    QThread::sleep(1);Stop(); end = _dataModel.GetPointFromString(_scanner.Scan()); QThread::usleep(200000);
    GoBack(12,12);
    QThread::sleep(1);Stop(); start = _dataModel.GetPointFromString(_scanner.Scan());QThread::usleep(200000);
    currentAngle = findAngle(start,end);
    if((finalAngle - currentAngle ) < 0){
        while((currentAngle - finalAngle) > 3){
        _AX12.SetPosition(3,100,12 - 1024);
        _AX12.SetPosition(1,100,17);
        QThread::sleep(1);Stop(); end = _dataModel.GetPointFromString(_scanner.Scan()); QThread::usleep(200000);
        GoBack(12,12);
        QThread::sleep(1);Stop(); start = _dataModel.GetPointFromString(_scanner.Scan());QThread::usleep(200000);
        currentAngle = findAngle(start,end);
        qDebug("Angle:"+QString::number(currentAngle).toLatin1());
        }
    }
    else{
        while((finalAngle - currentAngle) > 3){
        _AX12.SetPosition(3,100,17 - 1024);
        _AX12.SetPosition(1,100,12);
        QThread::sleep(1);Stop(); end = _dataModel.GetPointFromString(_scanner.Scan());QThread::usleep(200000);
        GoBack(12,12);
        QThread::sleep(1);Stop(); start = _dataModel.GetPointFromString(_scanner.Scan());QThread::usleep(200000);
        currentAngle = findAngle(start,end);
        qDebug("Angle:"+QString::number(currentAngle).toLatin1());
        }
    }
}

double Robot::findAngle(Point start,Point end){
    double angle = 0;

    if(start.x == end.x){
        if(start.y > end.y)
            return 270;
        else
            return 90;
    }
    if(start.y == end.y){
        if(start.x > end.x)
            return 180;
        else
            return 0;
    }
    angle = (end.y-start.y);
    angle /= (end.x-start.x);
    if(end.x < start.x){
        if(end.y < start.y)
        {
            angle = atan(angle);
            return 180 + (angle*(180/PINUM));
        }
        angle = atan(-1*angle);
        return 180 - (angle*(180/PINUM));
    }
    angle = atan(angle);
    if(angle < 0)
        return 360 + (angle*(180/PINUM));
    return angle*(180/PINUM);
}
 void Robot::Cleanup()
{
   Stop();
}

 Robot::~Robot()
{

    Stop();

}

