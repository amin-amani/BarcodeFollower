#ifndef SCANNER_H
#define SCANNER_H

#include <QObject>
#include <QDebug>
#include <QFile>
#include <QTimer>
#include <QSerialPort>
#include <QDateTime>
#include <QSignalSpy>
#include "datamodel.h"

#define TIME_OUT_THREDSHOLD 5

class Scanner : public QObject
{
    Q_OBJECT
private:
    QTimer recieveTimer;
   QSerialPort *_comport;
   QByteArray _buffer;
   Point result;
   unsigned int timerCounter;
public:
    explicit Scanner(QObject *parent = 0);
~Scanner();
   Point Scan();
   void Init();
   void Close();
   void reset();
signals:
   void DataReady(Point position);
   void Dummy();
   void StopRobot();

private slots:
   void ReadyRead();
   void RecieveTimeOut();
};

#endif // SCANNER_H
