//Id3 == right Motor , Min=0 , forward = positive - 1024, backward = postive number
//Id1 == left Motor forward = positive number, backward = positive - 1024
// max = 1023, min = 100 , center = 500
// git commit -am "message"
//git checkout -b branchname ex: git checkout -b 00001_TEST_BRACH

#include <QCoreApplication>
#include <QtDebug>
#include <QDebug>
#include "dynamixel.h"
#include "scanner.h"
#include "tcpserver.h"
#include "datamodel.h"
#include "controller.h"
#include "robot.h"
#include "xl320.h"
using namespace std;
Dynamixel ax12;
XL320 ax13;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    /*Scanner barCode;
    barCode.Init();
    for(int i = 0 ; i < 1000;i++){
        barCode.Scan();
        QThread::sleep(1);
    }*/

   // ax13.Init("/dev/ttyS2",115200);
//    ax13.SetPosition(1,0);
//    ax13.SetPosition(3,0);
//QThread::sleep(1);/*
//ax13.SetPosition(1,50);
//ax13.SetPosition(3,100);*/
//int i = 0;
//    while(1){
       // ax13.SetPosition(3,i*10);
//        qDebug(QString::number(ax13.GetSensor()).toLatin1());
//        ax13.SetPosition(1,100 + i);
//        ax13.SetPosition(3,100 + i);
//        QThread::sleep(1);
//        qDebug(QString::number(i).toLatin1());
//        QThread::sleep(1);
       // i+=20;
//    }
//    ax13.SetPosition(1,0);
//    ax13.SetPosition(3,0);


//Scanner s;
//s.Init();
    Robot robot;
//   ax13.Init("/dev/ttyS2",115200);
//   while(1){
////   ax13.SetPosition(1,500);
////   ax13.SetPosition(3,500);
//   }
   //ax12.SetTorqueEnable(1,false);
   //ax12.SetPosition(1,100,0);
    //robot.Stop();
   //qDebug(QString::number( ax12.ReadVoltage(1)).toLatin1());
//ax12.SetPosition(1,100,0);
//ax12.SetPosition(3,100,0);
//   ax12.SetPosition(1,100,100);
//   ax12.SetPosition(3,100,100);
//   int i = 0;
   /*
while(1){
    ax12.SetPosition(1,100,50);
    ax12.SetPosition(3,100,50);
if(i%8)
    ax12.GetSensor();
QThread::sleep(2);
i++;
}

*/
    return a.exec();
}
