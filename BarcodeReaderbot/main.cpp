//Id3 == right Motor , Min=0 , forward = positive - 1024, backward = postive number
//Id1 == left Motor forward = positive number, backward = positive - 1024

//git commit -am "message"
//git checkout -b 00001_BRQCNH1
//git push


#include <QCoreApplication>
#include <QtDebug>
#include <QDebug>
#include "dynamixel.h"
#include "scanner.h"
#include "tcpserver.h"
#include "datamodel.h"
#include "controller.h"
#include "robot.h"
using namespace std;
Dynamixel ax12;
//DataModel dataModel;
//bool lastState=false;
//int forwardSpeed=1034;
//int TargetX=100;
//int TargetY=100;
//Scanner scanner;

//void GoForward()
//{
//    ax12.SetPosition(3,100,forwardSpeed);
//    ax12.SetPosition(1,100,forwardSpeed-1024);

//}
//void GoBack()
//{
//    ax12.SetPosition(3,100,forwardSpeed-1024);
//    ax12.SetPosition(1,100,forwardSpeed);

//}
//void Stop()
//{
//    ax12.SetPosition(3,100,1024);
//    ax12.SetPosition(1,100,0);

//}

//void GetPositon()
//{
//  GoForward();
//QThread::sleep(1000);
//  Stop();
//   Point position= dataModel.GetPointFromString(scanner.Scan());
//    QThread::sleep(1000);
//  GoBack();
//  QThread::sleep(1000);
//  Stop();


//}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Robot robot;

    robot.Stop();
   // robot.GoForward();



  //  Robot *robot=new Robot();
//robot->GoBack();
    //printf("salam\n");
//TCPServer *server=new TCPServer(6000);
//qDebug("sa");
//qDebug()<<"hi";

//ax12.Init("/dev/ttyUSB0",1000000);
//ax12.SetPosition(3,100,0);
//ax12.SetPosition(1,100,0);
//QThread::sleep(1000);
//ax12.SetPosition(3,100,0);
//ax12.SetPosition(1,100,1024);
//ax12.SetLed(40,false);
//qDebug();
//ax12.Ping(40);

//ax12.SetPosition(3,100,1034);
//ax12.SetPosition(1,100,10);

//GetPositon();

//scanner.Scan();
//ax12.SetPosition(3,100,1034);
//ax12.SetPosition(1,100,10)

//while (false) {




//Point position= dataModel.GetPointFromString(scanner.Scan());
//qDebug(QString::number(position.x).toLatin1()+" "+QString::number(position.y).toLatin1());

//  //  int DiffX=TargetX-position.x;
//    //int DiffY=TargetY-position.y;

////qDebug(),DiffX;


//if(position.x<0){

// Stop();
// lastState=false;


//}

//else if (position.y<=140 && !lastState){
//   GoForward();

//}
//else if(position.y>=140){
//    GoBack();
//    lastState=true;
//}
//else if(position.y<=40 && lastState)
//{
//GoForward();
//lastState=false;
//}




//         }

    //qDebug()<<ax12.ReadPosition(40);
        return a.exec();
}
