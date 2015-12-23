#ifndef CONNECTIONTHREAD_H
#define CONNECTIONTHREAD_H

#include <QThread>

#include <QtNetwork>

class ConnectionThread: public QThread
{
    Q_OBJECT

public:
    ConnectionThread(int socketDescriptor, QObject *parent);

    void run() Q_DECL_OVERRIDE;

signals:
    void error(QTcpSocket::SocketError socketError);

public slots:
    void sendMsg(QString msg);
    void closeConnectionAndFinish();

private:
    int socketDescriptor;
    QTcpSocket tcpSocket;
   // bool initialized;
};

#endif // CONNECTIONTHREAD_H
