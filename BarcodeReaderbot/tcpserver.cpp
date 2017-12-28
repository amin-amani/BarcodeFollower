#include "tcpserver.h"

TCPServer::TCPServer(QObject *parent) : QObject(parent)
{
_connected=false;
}

TCPServer::TCPServer(int tcpPort){

_connected=false;
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
    _connected=true;


}
void TCPServer::SocketReadyRead(){

    QByteArray Data = _socket->readAll();

emit DataReady(Data);
}
void TCPServer::SocketDisconnected(){

    _socket->deleteLater();
    _connected=false;


}

void TCPServer::SendString(QString input){
    if(_connected)
    _socket->write(input.toLatin1());
}
bool TCPServer::Connected()
{
   return _connected;
}
