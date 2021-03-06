#include "scanner.h"

Scanner::Scanner(QObject *parent) : QObject(parent)
{

    _media.Init("/dev/ttyACM0",9600,2,QSerialPort::SoftwareControl);
    connect(&_media,SIGNAL(ReadyRead(QByteArray)),this,SLOT(ReadyRead(QByteArray)));
    QFile file("/sys/class/gpio/gpio21/value");
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
    exportFile.close();

}

void Scanner::ReadyRead(QByteArray data)
{

    QSignalSpy waitForNoting(this,SIGNAL(Dummy()));
     waitForNoting.wait(100);
    _buffer.insert(0,data);
    _media.ClearBuffer();
    emit Dummy();

}
QByteArray Scanner::Scan()
{
    QSignalSpy waitForNoting(this,SIGNAL(Dummy()));
    _buffer.clear();
     QFile file("/sys/class/gpio/gpio21/value");
    file.open(QFile::WriteOnly);
    file.write("1");
    file.flush();
    waitForNoting.wait(100);
    file.write("0");
    file.flush();
    file.close();
    waitForNoting.wait(400);
    return _buffer;

}
