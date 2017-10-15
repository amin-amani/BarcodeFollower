#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include <qt5/QtNetwork/QTcpSocket>
#include <qt5/QtNetwork/QTcpServer>
class TCPServer : public QObject
{
    Q_OBJECT
    QTcpSocket *_socket;
     QTcpServer _tcpServer;

public:
    explicit TCPServer(QObject *parent = 0);

     TCPServer(int tcpPort);
signals:

public slots:
private slots:
     void SocketReadyRead();
     void TcpServerNewConnection();
     void SocketDisconnected();
};

#endif // TCPSERVER_H
