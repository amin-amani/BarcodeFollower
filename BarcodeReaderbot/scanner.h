#ifndef SCANNER_H
#define SCANNER_H

#include <QObject>
#include <QFile>
#include "Media/media.h"

class Scanner : public QObject
{
    Q_OBJECT
private:
   Media _media;
   QByteArray _buffer;
public:
    explicit Scanner(QObject *parent = 0);

   QByteArray Scan();
signals:
   void Dummy();

public slots:
private slots:
   void ReadyRead(QByteArray data);
};

#endif // SCANNER_H
