#include "connectionthread.h"


#include <QtNetwork>

ConnectionThread::ConnectionThread(int socketDescriptor, QObject *parent)
    : QThread(parent), socketDescriptor(socketDescriptor)
{

    if (!tcpSocket.setSocketDescriptor(socketDescriptor)) {
        qDebug() << "ERROR";
        emit error(tcpSocket.error());
        return;
    }
    QString s = "Hola\n";
    tcpSocket.write(s.toLocal8Bit());
}

void ConnectionThread::run()
{
    qDebug() << "Cliente conectado!";

    QThread::exec();
}


void ConnectionThread::sendMsg(QString msg)
{
   // qDebug() << msg;
    if(tcpSocket.isOpen())
       tcpSocket.write(msg.toLocal8Bit());
    else
    {
        qDebug() << "La conexion esta cerrada. Terminando...";
        QThread::quit();
    }
}

void ConnectionThread::closeConnectionAndFinish()
{
    qDebug() << "Closing connection and finishing...";
    if(tcpSocket.isOpen())tcpSocket.close();
     qDebug() << "Connecion closed";
    QThread::quit();
}
