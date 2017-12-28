#include "scanner.h"

void Scanner::Init()
{
    //_media.Init(,,2,QSerialPort::SoftwareControl);

    result.x=-1;
    result.y=-1;
    timerCounter = 0;
    _comport = new QSerialPort();

    connect(&recieveTimer,SIGNAL(timeout()),this,SLOT(RecieveTimeOut()));
    recieveTimer.start(1000);

    _comport->setBaudRate(9600);
    _comport->setPortName("/dev/ttyS1");

 if(!_comport->open(QSerialPort::ReadWrite))
 {
     qDebug("barcode error open device");
 }
 else{
        connect(_comport,SIGNAL(readyRead()),this,SLOT(ReadyRead()));
        qDebug("barcode port connect ok");}
}

void Scanner::Close()
{
    _comport->close();
}

Scanner::Scanner(QObject *parent) : QObject(parent)
{


    /*QFile file("/sys/class/gpio/gpio21/value");
    if(file.exists())return;
    QFile exportFile("/sys/class/gpio/export");
    QFile directionFile("/sys/class/gpio/gpio21/direction");
    exportFile.open(QFile::WriteOnly);
    exportFile.write("21");
    exportFile.flush();
    QThread::sleep(1);
    directionFile.open(QFile::WriteOnly);
    directionFile.write("out");
    directionFile.close();
    exportFile.close();*/

}

Scanner::~Scanner()
{
//    qDebug("~scanner");
//    if(_comport->isOpen())
//    _comport->close();

}

void Scanner::ReadyRead()
{
    timerCounter = 0;
    QByteArray string;
    if(!_comport->canReadLine())
        return;
    string = _comport->readLine();
    QList<QByteArray> sp = string.trimmed().split(' ');
    if(sp.count() < 2)
        return ;
    result.x=sp[0].toInt();
    result.y=sp[1].toInt();
    //qDebug(" ");
  //qDebug("result x: " + QString::number(result.x).toLatin1());
    //qDebug("result y: " + QString::number(result.y).toLatin1());
//     qDebug(QDateTime::currentDateTime().time().toString().toLatin1()+
//           " result y: " + QString::number(result.y).toLatin1()+
//           " result x: " + QString::number(result.x).toLatin1());

}

void Scanner::RecieveTimeOut()
{
    if(timerCounter < TIME_OUT_THREDSHOLD)
        timerCounter++;
}

void Scanner::reset(){
    result.x = -1;
    result.y = -1;
}
Point Scanner::Scan()
{
    if(timerCounter >= TIME_OUT_THREDSHOLD)
    {
        result.x = -1;
        result.y = -1;
    }
    _comport->readAll();
    return result;

}
