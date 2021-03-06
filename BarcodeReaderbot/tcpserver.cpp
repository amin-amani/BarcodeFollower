#include "tcpserver.h"

TCPServer::TCPServer(QObject *parent) : QObject(parent)
{

}

TCPServer::TCPServer(int tcpPort){


    if(_tcpServer.listen(QHostAddress::Any,tcpPort)!=true){
        printf("Error on starting server on port:%d\n",tcpPort);
        return;
    }
   printf("Start server on port:%d\n",tcpPort);
    connect(&_tcpServer,SIGNAL(newConnection()),this,SLOT(TcpServerNewConnection()));

}
void TCPServer::TcpServerNewConnection(){

    _socket = _tcpServer.nextPendingConnection();

    connect(_socket,SIGNAL(readyRead()),this,SLOT(SocketReadyRead()));
    connect(_socket,SIGNAL(disconnected()),this,SLOT(SocketDisconnected()));


}
void TCPServer::SocketReadyRead(){

    QByteArray Data = _socket->readAll();
_socket->write("ans");
qDebug(Data.data());
}
void TCPServer::SocketDisconnected(){

    _socket->deleteLater();

}
