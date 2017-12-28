#include "robot.h"

Robot::Robot(QObject *parent) : QObject(parent)
{
   // _scanner.Init();

    controller = new Controller();
    //controllerThread = new QThread();
    //controller->moveToThread(controllerThread);
    //connect(controllerThread,SIGNAL(started()),controller,SLOT(Start()));

    //connect(controller,SIGNAL(WorkFinished()),controllerThread,SLOT(quit()));
    //..
    //connect(controllerThread,SIGNAL(finished()),controller,SLOT(deleteLater()));

    //connect(controllerThread,SIGNAL(finished()),controllerThread,SLOT(deleteLater()));


    connect(controller,SIGNAL(UpdatePoint(int,int,double,int,int)),this,SLOT(SendPositionToServer(int,int,double,int,int)));
    connect(this,SIGNAL(StopRobot()),controller,SLOT(StopRobot()));
    //connect(this,SIGNAL(StopRobot()),controller,SLOT(StopRobot()));


    kp = 0.1;
    kd = 0.5;
    ki = 0;

    //Stop();
    forwardSpeed=17;
//    _AX12.Init("/dev/ttyS2",115200);
//    _scanner.Init();
mission_stop = false;
mission_done = true;

    server = new TCPServer(6000);
connect(server,SIGNAL(DataReady(QByteArray)),this,SLOT(UpdateFinalPosition(QByteArray)),Qt::QueuedConnection);
connect(this,SIGNAL(GoSignal(Point,double)),this,SLOT(GotoPosition(Point,double)));
connect(this,SIGNAL(GoCircle(Point,int)),this,SLOT(Circle(Point,int)));
connect(this,SIGNAL(GoPathSignal(QQueue<Point>)),this,SLOT(FollowPath(QQueue<Point>)));



Point temp;
temp.x = 100;
temp.y = 100;

//GotoPoint(temp);
/*turnLeft(180);
QThread::sleep(2);
turnRight(180);
QThread::sleep(2);
turnLeft(180);
QThread::sleep(2);
turnRight(180);
for(int i =0; i< 10;i++){
GoForward(400,400);
QThread::sleep(5);
GoBack(400,400);
QThread::sleep(5);
turnLeft(50);
QThread::sleep(5);
GoForward(400,400);
QThread::sleep(5);
turnRight(50);
QThread::sleep(5);
GoBack(400,400);
QThread::sleep(5);
GoForward(400,400);
QThread::sleep(5);
GoBack(400,400);
QThread::sleep(5);
turnLeft(50);
QThread::sleep(5);
GoForward(400,400);
QThread::sleep(5);
turnRight(50);
QThread::sleep(5);
GoBack(400,400);
QThread::sleep(5);
Stop();
}*/
}

void Robot::UpdateFinalPosition(QByteArray position)
{
    QString point = QString::fromLatin1(position);
qDebug(point.toLatin1());
    if(point == "Stop"){
        emit StopRobot();
    }
    else if(point == "CheckHeading"){
        findCurrentAngle();
    }
    else if(mission_done && point.contains("Circle")){
        findCurrentAngle();
        Point temp;

        QStringList sp= point.trimmed().split(',');
        if(sp.length() > 1)
        {
            temp.x=sp[1].toInt();
            temp.y=sp[2].toInt();
            int radius = sp[3].toInt();
            emit GoCircle(temp,radius);
        }
    }
    else if(mission_done && point.contains("Path")){
        //findCurrentAngle();
        mission_stop = false;
        QQueue<Point> path;

        QStringList sp= point.trimmed().split(' ');
        kp = sp[1].trimmed().split(',')[0].toDouble();
        ki = sp[1].trimmed().split(',')[1].toDouble();
        kd = sp[1].trimmed().split(',')[2].toDouble();
        int i = 2;
        if(sp.length() > 1)
        {
            while(sp[i] != "EndPath"){
                Point temp;
                QStringList pointString= sp[i].trimmed().split(',');
                if(pointString.length() > 1){
                    temp.x=pointString[0].toInt();
                    temp.y=pointString[1].toInt();
                }

                if(i < (sp.length() - 1))
                {
                    path.enqueue(temp);
                    i++;
                }
                else
                    break;
            }
qDebug("omade 1");
            emit GoPathSignal(path);
        }
    }
    else if(mission_done){
        Point temp;
        findCurrentAngle();

        QStringList sp= point.trimmed().split(',');
        if(sp.length() > 1)
        {
            temp.x=sp[0].toInt();
            temp.y=sp[1].toInt();
            emit GoSignal(temp,sp[2].toInt());
        }
    }
}

void Robot::SendPositionToServer(int x, int y, double angle,int x_carrot,int y_carrot)
{
    server->SendString(QString::number(x).toLatin1()+","+QString::number(y).toLatin1()+","+QString::number(angle).toLatin1()+","+QString::number(x_carrot).toLatin1()+","+QString::number(y_carrot).toLatin1()+"\n");
}
void Robot::GoForwardR(int speedR)
{
    forward = true;
    //_AX12.SetPosition(3,100,speedR - 1024);
}

void Robot::GoForwardL(int speedL)
{
    //_AX12.SetPosition(1,100,speedL);
}

void Robot::GoForward(int speedR, int speedL)
{
    forward = true;
    //_AX12.SetPosition(3,100,speedR - 1024);
    //_AX12.SetPosition(1,100,speedL);
    //_AX12.SetPosition(3,speedR + 20);
    //_AX12.SetPosition(1,speedL + 1033);

}
void Robot::GoBackR(int speedR)
{
    //_AX12.SetPosition(3,100,speedR);
}
void Robot::GoBackL(int speedL)
{
    //_AX12.SetPosition(1,100,speedL - 1024);
}
void Robot::GoBack(int speedR, int speedL)
{
    forward = false;
    //_AX12.SetPosition(3,100,speedR);
    //_AX12.SetPosition(1,100,speedL - 1024);
    //_AX12.SetPosition(3,speedR + 1043);
    //_AX12.SetPosition(1,speedL + 13);

}
void Robot::Stop()
{
    //_AX12.SetPosition(3,100,1024);
    //_AX12.SetPosition(1,100,0);
//    _AX12.SetPosition(3,0);
//    _AX12.SetPosition(1,0);
//    _AX12.SetTorqueEnable(3,0);
//    _AX12.SetTorqueEnable(1,0);

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
    double err = 0;
    double derivative = 0;


    err = setpoint - actualValue;
    integral = integral + err;
    derivative = (err - preError);
    preError=err;
    result = (Kp * err);
    result += (Ki * integral);
    result += (Kd * derivative);
    if(integral>max)integral=max;
    else if(integral<min)integral=min;
    if(result>max)return max;
    if(result<min)return min;
    return result;
}

Point Robot::GetCurrentPosition(){
    qDebug("Current Pos");
    double tempAngle = 0;
    Point result;
    //result = _scanner.Scan();
    if(result.x == -1)
        return result;
    /*if(result.x < BORDER_X_MIN || result.x > BORDER_X_MAX || result.y < BORDER_Y_MIN || result.y > BORDER_X_MAX )
    {
        mission_stop = true;
    }*/
    if(GetDistancePoint(lastPos,result) > 2)
    {
        if(forward)
            tempAngle = findAngle(lastPos,result);
        else
            tempAngle = findAngle(result,lastPos);
        currentAngle = tempAngle;
        //qDebug("(" + QString::number(lastPos.x).toLatin1() + "," + QString::number(lastPos.y).toLatin1() +") (" + QString::number(result.x).toLatin1() + "," + QString::number(result.y).toLatin1() +") Angle " + QString::number(tempAngle).toLatin1() + (forward?" F":" B"));
        lastPos = result;
    }
    //qDebug("(" + QString::number(lastPos.x).toLatin1() + "," + QString::number(lastPos.y).toLatin1() +") (" + QString::number(result.x).toLatin1() + "," + QString::number(result.y).toLatin1() +") Angle " + QString::number(tempAngle).toLatin1());
    server->SendString(QString::number(result.x).toLatin1()+","+QString::number(result.y).toLatin1()+","+QString::number(tempAngle).toLatin1()+"\n");

    return lastPos;
}

Point Robot::GetExactCurrentPosition(){
    double tempAngle = 0;
    Point result;
    //result = _scanner.Scan();
    while(result.x == -1){
        qDebug("Exc Current Pos");
        //result = _scanner.Scan();
        QThread::msleep(2);
    }

    /*if(result.x < BORDER_X_MIN || result.x > BORDER_X_MAX || result.y < BORDER_Y_MIN || result.y > BORDER_X_MAX )
    {
        mission_stop = true;
    }*/
    if(GetDistancePoint(lastPos,result) > 2)
    {
        if(forward)
            tempAngle = findAngle(lastPos,result);
        else
            tempAngle = findAngle(result,lastPos);
        currentAngle = tempAngle;
        //qDebug("(" + QString::number(lastPos.x).toLatin1() + "," + QString::number(lastPos.y).toLatin1() +") (" + QString::number(result.x).toLatin1() + "," + QString::number(result.y).toLatin1() +") Angle " + QString::number(tempAngle).toLatin1() + (forward?" F":" B"));
        lastPos = result;
    }
    //qDebug("(" + QString::number(lastPos.x).toLatin1() + "," + QString::number(lastPos.y).toLatin1() +") (" + QString::number(result.x).toLatin1() + "," + QString::number(result.y).toLatin1() +") Angle " + QString::number(tempAngle).toLatin1());
    server->SendString(QString::number(result.x).toLatin1()+","+QString::number(result.y).toLatin1()+","+QString::number(tempAngle).toLatin1()+"\n");

    return lastPos;
}


void Robot::Circle(Point center,int radius){
    mission_done = false;
    PIDInit();
    Point currentPos;
    Point start = GetCurrentPosition();
    Line line = getLineFunc(start,center);
    currentPos = start;
    double currentDis = 0;
    int max = 20;
    int motorOffset = 30;
    int min = -20;
    Stop();
    QSignalSpy waitForNoting(this,SIGNAL(Dummy()));
    while((GetDistancePoint(center,currentPos)  > radius) && !mission_stop){

        qDebug("Dis:"+QString::number(GetDistancePoint(center,currentPos)).toLatin1());
        qDebug("Goal("+QString::number(center.x).toLatin1()+","+QString::number(center.y).toLatin1()+")");
        qDebug("Current("+QString::number(currentPos.x).toLatin1()+","+QString::number(currentPos.y).toLatin1()+")");
        currentPos = GetCurrentPosition();
        currentDis = GetDistance(line,currentPos);
        double pid = PIDControl(0,whichSide(start,center,currentPos)*currentDis,1,kp,ki,kd,max,min);
        GoForward(motorOffset - pid ,motorOffset + pid);
        qDebug("pid: " + QString::number(pid).toLatin1());

        waitForNoting.wait(1);
    }
    Stop();
    PIDInit();
    Stop();
    QSignalSpy waitForNoting1(this,SIGNAL(Dummy()));
    while(!mission_stop){
        currentPos = GetCurrentPosition();
        currentDis = GetDistancePoint(currentPos,center);
        double pid = PIDControl(radius,currentDis,1,kp,ki,kd,max,min);
        GoForward(motorOffset - pid ,motorOffset + pid);
        qDebug("Dis:"+QString::number(currentDis).toLatin1());
        qDebug("Goal("+QString::number(center.x).toLatin1()+","+QString::number(center.y).toLatin1()+") "+QString::number(radius).toLatin1());
        qDebug("pid: " + QString::number(pid).toLatin1());
        waitForNoting1.wait(1);
    }
    Stop();
    mission_done = true;
    mission_stop = false;
}

void Robot::FollowPath(QQueue<Point> path)
{
//    qDebug("follow path");
//    mission_done = false;
//    while(!mission_stop && (path.size() > 0)){
//        qDebug("follow path while");
//        Point temp = path.dequeue();
        controller->Init(path,kp,kd,ki);
        controller->Start();
//    }
//    mission_done = true;
//    mission_stop = false;
}

double Robot::magnitude(Point AB){
    double result = 0;
    result = (AB.x*AB.x);
    result += (AB.y*AB.y);
    result = sqrt(result);
    return result;
}
void Robot::GotoPoint(Point goal){



}

void Robot::GotoPosition(Point goal,double finalAngle)
{
    mission_done = false;
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

//    checkHeading(mid_goal);
    GotoPoint(mid_goal);
  //  checkHeading(goal);
    GotoPoint(goal);
    mission_done = true;
    mission_stop = false;
}

void Robot::checkHeading(Point end){
    Point start;
    start = GetExactCurrentPosition();

    double finalAngle =  findAngle(start,end);
    qDebug("Goal("+QString::number(end.x).toLatin1()+","+QString::number(end.y).toLatin1()+")"+" currentAngle("+QString::number(currentAngle).toLatin1()+")"+" finalAngle("+QString::number(finalAngle).toLatin1()+")");
    //QThread::sleep(10);
    //qDebug("current Angle:"+QString::number(currentAngle).toLatin1());
    //qDebug("final Angle:"+QString::number(finalAngle).toLatin1());
    turnRobot(finalAngle);
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

void Robot::turnRight(unsigned long val){
    //_AX12.SetPosition(3,100,17);
    //_AX12.SetPosition(1,100,17);
    //_AX12.SetPosition(3,220);
    //_AX12.SetPosition(1,200);
    QThread::msleep(val*12);Stop();
}
void Robot::turnLeft(int val){
   // _AX12.SetPosition(3,100,17 - 1024);
   // _AX12.SetPosition(1,100,17 - 1024);
    //_AX12.SetPosition(3,220 + 1023);
    //_AX12.SetPosition(1,200 + 1023);
    QThread::msleep(val*12);Stop();
}
void Robot::findCurrentAngle(){
    GoForward(150,150);
    QThread::sleep(2);Stop();Point end = GetCurrentPosition();
    GoBack(150,150);
    QThread::sleep(2);Stop();Point start = GetCurrentPosition();
    currentAngle = findAngle(start,end);
    //qDebug("Angle:"+QString::number(currentAngle).toLatin1());
}
void Robot::turnRobot(double finalAngle){
    if(CHECKANGLE)
        findCurrentAngle();

    //do{
    if((finalAngle - currentAngle ) < -180){
                turnLeft(currentAngle-finalAngle);
    }
    else if((finalAngle - currentAngle ) < 0){
            turnRight(currentAngle-finalAngle);
    }
    else if((finalAngle - currentAngle ) > 180){
            turnRight(finalAngle - currentAngle);
    }
    else{
            turnLeft(finalAngle - currentAngle);
    }
    Stop();
    //findCurrentAngle();
    //}
    //while(abs(currentAngle - finalAngle) > 5);
    qDebug("Stop");
    QThread::usleep(1000);
}

double Robot::findAngle(Point start,Point end){
    /*double tetha = 0;
    Point AB;
    AB.x = end.x - start.x;
    AB.y = end.y - end.x;
    Point CD;
    CD.x = 1;
    CD.y = 0;
    double magA = magnitude(AB);
    //qDebug("Mag A: "+QString::number(magA).toLatin1());
    double magC = 1;
    //qDebug("Mag B: "+QString::number(magC).toLatin1());
    tetha = dotProduct(AB,CD);
    //qDebug("Dot AB CD: "+QString::number(tetha).toLatin1());
    tetha /= magA;
    tetha /= magC;
    tetha = acos(tetha);
    //qDebug("Tetha: "+QString::number(tetha*180/3.1415).toLatin1());
    //double result = 0;
   // result = magA * magC;
    //result = sin(tetha/2);
    //qDebug("Cross: "+QString::number(result).toLatin1());
    double signResult = 1;
    signResult = (AB.x * CD.y);
    signResult -= (AB.y * CD.x);

    tetha *= 180;
    tetha /= 3.14159265;
    if(signResult >= 0)
        return tetha;
    else
        return (360 - tetha);*/
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

  //  Stop();

}

