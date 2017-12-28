#include "controller.h"

Controller::Controller()
{
    mission_done = true;
    mission_stop = false;
    lastPos.x = 0;
    lastPos.y = 0;
    forward = true;

    maxPID = 100;
    motorOffsetPID = 200;
    minPID = -100;
    readErrorCounterPID = 0;

    _scanner.Init();

    _AX12 = new XL320();
    _AX12->Init("/dev/ttyS2",115200);
    controllerTimer = new QTimer();
    serverDataTimer = new QTimer();



    connect(controllerTimer,SIGNAL(timeout()),this,SLOT(startPID()));
    connect(serverDataTimer,SIGNAL(timeout()),this,SLOT(sendServerData()));
    connect(this,SIGNAL(WorkFinished()),this,SLOT(StopRobot()));

   // _AX12 = new XL320();
    //_AX12->Init("/dev/ttyS2",115200);
    qDebug("constructor controller");


    //mai thread
    connect(&updateTimer,SIGNAL(timeout()),this,SLOT(UpdateStatus()));
    updateTimer.start(100);
}

Controller::~Controller()
{
    qDebug("Destruct");
  //  _scanner.Close();
}

void Controller::Init(QQueue<Point> goal,double kp, double kd, double ki)
{
    Goals = goal;
    _kp = kp;
    _kd = kd;
    _ki = ki;
}

void Controller::Start()
{
    mission_done = true;
    mission_stop = false;

    currentPosPID = currentPos;
    StartPID = currentPos;

    if(!findAngleOffset())
    {
        qDebug("Sorry!");
        return;
    }
    PIDInit();
    controllerTimer->start(500);
    serverDataTimer->start(1000);
}

bool Controller::findAngleOffset(){
    if(!findCurrentAngle(2))
        return false;
    unsigned int sensorValue;
    if(!_AX12->GetSensor(&sensorValue))
        return false;
    double sensorAngle = (360 - sensorValue);
    angle_offset =  sensorAngle - currentAngle;
    qDebug("Barcode Angle: " + QString::number(currentAngle).toLatin1());
    qDebug("Sensor Angle: " + QString::number(sensorAngle).toLatin1());
}

double Controller::SensorAngle(){
    //QMutexLocker lock(&dataMutex);
    double sensorAngle ;
    unsigned int sensorValue;
    if(!_AX12->GetSensor(&sensorValue))
        return -1;
    sensorAngle = 360 - sensorValue;
   // lock.unlock();
   // if((360 - tempAngle - angle_offset ) > 0)
    double result = cos(sensorAngle*(PINUM/180));
    double result1 = sin(sensorAngle*(PINUM/180));
    result *= cos(angle_offset*(PINUM/180));
    result1 *= sin(angle_offset*(PINUM/180));
    double cosTheta = result + result1;

    result = cos(sensorAngle*(PINUM/180));
    result1 = sin(sensorAngle*(PINUM/180));

    result *= sin(angle_offset*(PINUM/180));
    result1 *= cos(angle_offset*(PINUM/180));
    double sinTheta = result - result1;

    if(sinTheta < 0 && cosTheta > 0)
        currentAngle = (acos(cosTheta)*((double)180/PINUM));
    else if(sinTheta > 0 && cosTheta > 0)
        currentAngle = (360 - (asin(sinTheta)*((double)180/PINUM)));
    else if(sinTheta > 0 && cosTheta < 0)
        currentAngle = (180 + asin(sinTheta)*((double)180/PINUM));
    else if(sinTheta < 0 && cosTheta < 0)
        currentAngle = (180 + asin(sinTheta)*((double)180/PINUM));
    return currentAngle;
  //  else

}

void Controller::StopRobot()
{
    mission_stop = true;
    controllerTimer->stop();
    Stop();
}

void Controller::startPID()
{
    if(Goals.count() >= 1)
        _goal = Goals.first();

    //while((GetDistancePoint(currentPos,goal) > 5) && ((currentPos.x > (qMin(start.x,goal.x)-DISAREA)) && (currentPos.x < (qMax(start.x,goal.x)+DISAREA)) && (currentPos.y > (qMin(start.y,goal.y)-DISAREA)) && (currentPos.y < (qMax(start.y,goal.y)+DISAREA))) && !mission_stop){
    if((GetDistancePoint(currentPosPID,_goal) > DIS_TO_GOAL) &&
          ((currentPosPID.x > (qMin(StartPID.x,_goal.x)-DISAREA)) &&
          (currentPosPID.x < (qMax(StartPID.x,_goal.x)+DISAREA)) &&
          (currentPosPID.y > (qMin(StartPID.y,_goal.y)-DISAREA)) &&
          (currentPosPID.y < (qMax(StartPID.y,_goal.y)+DISAREA))) &&
          !mission_stop && (Goals.count() >= 1)){
        Line line = getLineFunc(StartPID,_goal);

        Point currentPosTemp = currentPos;

        if(currentPosTemp.x == -1)
            readErrorCounterPID++;
        else
        {
            readErrorCounterPID = 0;
            currentPosPID = currentPosTemp;
        }
        getCarrotPosition(currentPosPID,line,StartPID,_goal);

        double finalAngle = findAngle(currentPosPID,carrot);
        double pid = 0;
        if((finalAngle - currentAngle) > 180)
            pid = PIDControl(finalAngle,360 + currentAngle,1,_kp,_ki,_kd,maxPID,minPID);
        else
            pid = PIDControl(finalAngle,currentAngle,1,_kp,_ki,_kd,maxPID,minPID);
        if(readErrorCounterPID > READ_ERROR_THRESHOLD)
            Stop();
        else
        {
           // QMutexLocker lock(&dataMutex);
            GoForward(motorOffsetPID - pid,motorOffsetPID + pid);
           // lock.unlock();
        }
        qDebug(QString::number(QDateTime::currentDateTime().time().second()).toLatin1() + " " +
                        QString::number(QDateTime::currentDateTime().time().msec()).toLatin1() +
               " Ang: "+QString::number(currentAngle).toLatin1()+
               " Start("+QString::number(StartPID.x).toLatin1()+
               ","+QString::number(StartPID.y).toLatin1()+")"+
               " Goal("+QString::number(_goal.x).toLatin1()+
               ","+QString::number(_goal.y).toLatin1()+")"+
               " pid: " + QString::number(pid).toLatin1()+
                        " Final Ang: "+QString::number(finalAngle).toLatin1());

    }
    else
    {
        StartPID = _goal;
        if(Goals.count() >= 1)
            Goals.dequeue();
        else
            emit WorkFinished();
        qDebug("==============================================================");
    }

}

//void Controller::startPID()
//{
//    //while((GetDistancePoint(currentPos,goal) > 5) && ((currentPos.x > (qMin(start.x,goal.x)-DISAREA)) && (currentPos.x < (qMax(start.x,goal.x)+DISAREA)) && (currentPos.y > (qMin(start.y,goal.y)-DISAREA)) && (currentPos.y < (qMax(start.y,goal.y)+DISAREA))) && !mission_stop){
//    while((GetDistancePoint(currentPosPID,_goal) > DIS_TO_GOAL) &&
//          ((currentPosPID.x > (qMin(StartPID.x,_goal.x)-DISAREA)) &&
//          (currentPosPID.x < (qMax(StartPID.x,_goal.x)+DISAREA)) &&
//          (currentPosPID.y > (qMin(StartPID.y,_goal.y)-DISAREA)) &&
//          (currentPosPID.y < (qMax(StartPID.y,_goal.y)+DISAREA))) &&
//          !mission_stop){
//      // if(1){
//        //Point currentPosTemp = GetCurrentPosition();
//        double angle = SensorAngle();
//       // qDebug(QString::number(angle).toLatin1());
//        //qDebug(QString::number(QDateTime::currentDateTime().time().second()).toLatin1() + " " + QString::number(QDateTime::currentDateTime().time().msec()).toLatin1() + "(" + QString::number(currentPosTemp.x).toLatin1() + "," + QString::number(currentPosTemp.y).toLatin1() + ")");

////        if(currentPosTemp.x == -1)
////            readErrorCounterPID++;
////        else
////        {
////            readErrorCounterPID = 0;
////            currentPosPID = currentPosTemp;
////        }
//        //currentDisPID = GetDistancePoint(currentPosPID,_goal);
//        double pid = PIDControl(finalAngle,angle,1,_kp,_ki,_kd,maxPID,minPID);
////        if(readErrorCounterPID > READ_ERROR_THRESHOLD)
////            Stop();
////        else
//            GoForward(motorOffsetPID - pid,motorOffsetPID + pid);
//        qDebug(QString::number(QDateTime::currentDateTime().time().second()).toLatin1() + " " +
//                QString::number(QDateTime::currentDateTime().time().msec()).toLatin1() +
//                " pid: " + QString::number(pid).toLatin1()+
//                " Ang: "+QString::number(angle).toLatin1()+
//                ") Final Ang: "+QString::number(finalAngle).toLatin1()+
//                " Start("+QString::number(StartPID.x).toLatin1()+
//                ","+QString::number(StartPID.y).toLatin1()+")");

//    }
////    else
////    {
//        emit WorkFinished();
//        qDebug("==============================================================");
//  //  }

//}

void Controller::sendServerData()
{
    Point result = _scanner.Scan();
    emit UpdatePoint(result.x,result.y,currentAngle,carrot.x,carrot.y);
}

void Controller::GoForward(int speedR, int speedL,int offsetR, int offsetL)
{
    forward = true;
    _AX12->SetPosition(3,speedR + offsetR);
    _AX12->SetPosition(1,speedL + offsetL + 1023);

}
void Controller::GoBack(int speedR, int speedL,int offsetR, int offsetL)
{
    forward = false;
    _AX12->SetPosition(3,speedR + offsetR + 1023);
    _AX12->SetPosition(1,speedL + offsetL);

}
void Controller::Stop()
{
    _AX12->SetPosition(3,0);
    _AX12->SetPosition(1,0);
    _AX12->SetTorqueEnable(3,0);
    _AX12->SetTorqueEnable(1,0);

}

Line Controller::getLineFunc(Point Start,Point End)
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

int Controller::whichSide(Point A,Point B,Point goal){
   double result = ((goal.x-A.x)*(B.y-A.y)-(goal.y-A.y)*(B.x-A.x));
   return (result>0?1:-1);
}
double Controller::GetDistancePoint(Point start,Point end){
    double result = 0;
    result = (end.x-start.x)*(end.x-start.x) + (end.y-start.y)*(end.y-start.y);
    return sqrt(result);
}
double Controller::GetDistance(Line line,Point point)
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
void Controller::PIDInit(){
    preError = 0;
    integral = 0;
}
double Controller::PIDControl(int setpoint,double actualValue,int Dt,double Kp,double Ki,double Kd,int max, int min)//about 10 us
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

Point Controller::GetCurrentPosition(){
   // qDebug("Current Pos");
    double tempAngle = 0;
    Point result;
    result = _scanner.Scan();
    QThread::msleep(50);
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
        qDebug("(" + QString::number(lastPos.x).toLatin1() + "," + QString::number(lastPos.y).toLatin1() +") (" + QString::number(result.x).toLatin1() + "," + QString::number(result.y).toLatin1() +") Angle " + QString::number(tempAngle).toLatin1() + (forward?" F":" B"));
        lastPos = result;
    }
    //qDebug("(" + QString::number(lastPos.x).toLatin1() + "," + QString::number(lastPos.y).toLatin1() +") (" + QString::number(result.x).toLatin1() + "," + QString::number(result.y).toLatin1() +") Angle " + QString::number(tempAngle).toLatin1());
    //server->SendString(QString::number(result.x).toLatin1()+","+QString::number(result.y).toLatin1()+","+QString::number(tempAngle).toLatin1()+"\n");
    emit UpdatePoint(result.x,result.y,currentAngle,carrot.x,carrot.y);
    return lastPos;
}
void Controller::checkHeading(Point end){
    Point start;
    start = GetCurrentPosition();

    double finalAngle =  findAngle(start,end);
    qDebug("Goal("+QString::number(end.x).toLatin1()+","+QString::number(end.y).toLatin1()+")"+" currentAngle("+QString::number(currentAngle).toLatin1()+")"+" finalAngle("+QString::number(finalAngle).toLatin1()+")");
    turnRobot(finalAngle);
    //mission_stop = true;
}
void Controller::PrintPoint(QString msg, Point point)
{
    qDebug(msg.toLatin1()+"("+QString::number(point.x).toLatin1()+","+QString::number(point.y).toLatin1()+")");
}
bool Controller::findCurrentAngle(int numberOfTry){
QSignalSpy waitForNoting(this,SIGNAL(Dummy()));

    while (numberOfTry > 0) {
        _scanner.reset();
        GoForward(120,110);
        //QThread::sleep(2);
        waitForNoting.wait(2000);
        Stop();
        waitForNoting.wait(1000);
        Point end = currentPos;
        waitForNoting.wait(2000);
        _scanner.reset();
        GoBack(120,100);
        waitForNoting.wait(2000);
        Stop();
        waitForNoting.wait(1000);
        Point start = currentPos;
        //QThread::sleep(1);
        waitForNoting.wait(2000);
        currentAngle = findAngle(start,end);
        //qDebug("Start("+QString::number(start.x).toLatin1()+","+QString::number(start.y).toLatin1()+") End("+QString::number(end.x).toLatin1()+","+QString::number(end.y).toLatin1()+") Angle:"+QString::number(currentAngle).toLatin1() );
        if((end.x != -1) && (start.x != -1))
           {
        if((start.x != end.x && start.y != end.y)) return true;
        }
            numberOfTry--;
    }
    return false;
}
void Controller::turnRobot(double finalAngle){
//    if(CHECKANGLE)
//    {
//        if(!findCurrentAngle(2))
//            return;
//    }

    //do{
//    if((finalAngle - currentAngle ) < -180){
//                turnRight(currentAngle-finalAngle);
//    }
//    else if((finalAngle - currentAngle ) < 0){
//            turnRight(currentAngle-finalAngle);
//    }
//    else if((finalAngle - currentAngle ) > 180){
//            turnLeft(finalAngle - currentAngle);
//    }
//    else{
//            turnLeft(finalAngle - currentAngle);
//    }

    if((finalAngle - currentAngle ) < 0){
        turnRight(currentAngle - finalAngle);
    }
    else{
        turnLeft(finalAngle - currentAngle);
    }
    Stop();
    QThread::usleep(1000);
}
double Controller::findAngle(Point start,Point end){
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
void Controller::turnRight(unsigned long val){
    //_AX12->SetPosition(3,100,17);
    //_AX12->SetPosition(1,100,17);
    _AX12->SetPosition(3,220);
    _AX12->SetPosition(1,200);
    QThread::msleep(val*12.7);Stop();
}
void Controller::turnLeft(unsigned long  val){
   // _AX12->SetPosition(3,100,17 - 1024);
   // _AX12->SetPosition(1,100,17 - 1024);
    _AX12->SetPosition(3,220 + 1023);
    _AX12->SetPosition(1,200 + 1023);
    QThread::msleep(val*13);Stop();
}

void Controller::getCarrotPosition(Point present,Line line,Point start,Point goal){
    carrot.x = line.a * present.y;
    carrot.x += present.x;
    carrot.x -= (line.a*line.c);
    carrot.x /= (line.a*line.a + 1);
    if((goal.x > start.x) && (goal.y > start.y))
    {
        carrot.x += (CARROT_OFFSET/sqrt(line.a*line.a+1));
        carrot.y = line.a*carrot.x + line.c;
    }
    else if((goal.x < start.x) && (goal.y > start.y))
    {
        carrot.x -= (CARROT_OFFSET/sqrt(line.a*line.a+1));
        carrot.y = line.a*carrot.x + line.c;
    }
    else if((goal.x < start.x) && (goal.y < start.y))
    {
        carrot.x -= (CARROT_OFFSET/sqrt(line.a*line.a+1));
        carrot.y = line.a*carrot.x + line.c;
    }
     else if((goal.x > start.x) && (goal.y < start.y))
    {
        carrot.x += (CARROT_OFFSET/sqrt(line.a*line.a+1));
        carrot.y = line.a*carrot.x + line.c;
    }
    else if((goal.x == start.x) && (goal.y > start.y))
    {
        carrot.x = goal.x;
        carrot.y = start.y + CARROT_OFFSET;
    }
    else if((goal.x == start.x) && (goal.y < start.y))
    {
        carrot.x = goal.x;
        carrot.y = start.y - CARROT_OFFSET;
    }
    else if((goal.y == start.y) && (goal.x > start.x))
    {
        carrot.y = goal.y;
        carrot.x = start.x + CARROT_OFFSET;
    }
    else if((goal.y == start.y) && (goal.x < start.x))
    {
        carrot.y = goal.y;
        carrot.x = start.x - CARROT_OFFSET;
    }
}

void Controller::UpdateStatus()
{
  currentPos= _scanner.Scan();

  double tempAngle = SensorAngle();
  if(tempAngle > 0)
      currentAngle = tempAngle;
 // lock.unlock();
//PrintPoint( "location=",currentPos);
//qDebug( QString::number(currentAngle).toLatin1());
emit UpdatePoint(currentPos.x,currentPos.y,currentAngle,carrot.x,carrot.y);
}

