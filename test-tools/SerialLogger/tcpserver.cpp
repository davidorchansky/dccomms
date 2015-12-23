#include "tcpserver.h"
#include "connectionthread.h"

TCPServer::TCPServer(QObject * parent): QTcpServer(parent)
{

}

void TCPServer::incomingConnection(qintptr socketDescriptor)
{
    ConnectionThread *thread = new ConnectionThread(socketDescriptor, this);
    connect(this, SIGNAL(newMsg(QString)), thread, SLOT(sendMsg(QString)));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(closeConnectionAndFinish()));
    connect(this, SIGNAL(closed()), thread, SLOT(closeConnectionAndFinish()));
    thread->start();
}


