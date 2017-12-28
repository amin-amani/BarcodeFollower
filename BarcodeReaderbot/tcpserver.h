#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <QObject>
#include <QThread>
#include <QDebug>
//#include <qt5/QtNetwork/QTcpSocket>
//#include <qt5/QtNetwork/QTcpServer>
#include <QTcpSocket>
#include <QTcpServer>
class TCPServer : public QObject
{
    Q_OBJECT
    QTcpSocket *_socket;
     QTcpServer _tcpServer;
     bool _connected;


public:
    explicit TCPServer(QObject *parent = 0);

     TCPServer(int tcpPort);
     void SendString(QString input);
     bool Connected();
signals:
 void DataReady(QByteArray);
public slots:
private slots:
     void SocketReadyRead();
     void TcpServerNewConnection();
     void SocketDisconnected();
};

#endif // TCPSERVER_H
