#ifndef LOGGERWINDOW_H
#define LOGGERWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTextCursor>
#include <tcpserver.h>

namespace Ui {
class LoggerWindow;
}

class LoggerWindow : public QMainWindow
{
    Q_OBJECT
    //Capturador *capturador;
    QSerialPort serial;
    QTextCursor textCursor;
public:
    explicit LoggerWindow(QWidget *parent = 0);
    ~LoggerWindow();
    
private slots:
    void on_capturarButton_clicked();
    void capturar();
    void pararCaptura();

    void on_guardarButton_clicked();
    void on_netejarButton_clicked();

    void on_fileBrowserButton_clicked();

    void on_filtreLineEdit_textChanged(const QString &arg1);

    void on_pushButton_clicked();

    void on_tcpCheckBox_toggled(bool checked);

public slots:
    void actualizarMonitor(QByteArray);
    void errorCritic(QString);
    void datosEnBuffer();
private:
    Ui::LoggerWindow *ui;
    QRegExp filtro;
    bool filtroActivado;
    QString acumulado;
    void comprobarFiltro();
    TCPServer tcpServer;
    bool sinPerdidas;
signals:
    void _pararCaptura();
    void nuevoMensaje(QString msg);

};

#endif // LOGGERWINDOW_H
