#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <QtNetwork>

class TCPServer: public QTcpServer
{
    Q_OBJECT

public:
    TCPServer(QObject *parent = 0);

signals:
    void newMsg(QString);
    void closed();

protected:
    void incomingConnection(qintptr socketDescriptor) Q_DECL_OVERRIDE;

private:
};

#endif // TCPSERVER_H
