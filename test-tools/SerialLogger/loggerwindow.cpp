#include "loggerwindow.h"
#include "ui_loggerwindow.h"
#include <QString>
#include<QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QFile>
#include <QMessageBox>
#include <QTextCodec>
#include <QFileDialog>
#include <QInputDialog>
#include <QDesktopServices>

LoggerWindow::LoggerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LoggerWindow)
{
    ui->setupUi(this);
    ui->monitorTextEdit->setReadOnly(true);
    textCursor = ui->monitorTextEdit->textCursor();
    ui->monitorTextEdit->ensureCursorVisible();

    connect(&serial,SIGNAL(readyRead()),this,SLOT(datosEnBuffer()));
    connect(this,SIGNAL(nuevoMensaje(QString)),&tcpServer,SIGNAL(newMsg(QString)));

    if(ui->tcpCheckBox->isChecked())
    {
        //Crear servidor TCP en port especificat.
        int port;
        QRegExp reg("^\\d+$");
        QString sp = ui->tcpPortLineEdit->text();
        if(reg.indexIn(sp) != -1)
            port = sp.toInt();
        else
            port = 8001;

        qDebug() << port;

        ui->tcpPortLineEdit->setText(QString::number(port));
        tcpServer.listen(QHostAddress::LocalHost,port);
    }
    sinPerdidas = ui->sinPerdidasCheckBox->isChecked();
    ui->sinPerdidasCheckBox->setEnabled(ui->filtreLineEdit->text().length()>0);
}

LoggerWindow::~LoggerWindow()
{
    delete ui;
}

void LoggerWindow::capturar()
{
    ui->capturarButton->setText("Parar");
    ui->portLineEdit->setDisabled(true);
    ui->baudrateLineEdit->setDisabled(true);
    ui->nLiniesLineEdit->setText("0");
    comprobarFiltro();
  //  capturador->start();
    serial.setPortName(ui->portLineEdit->text());
    if(! serial.open(QIODevice::ReadWrite))
    {
        errorCritic("ERROR: Impossible accedir al port especificat");
        pararCaptura();
        return;
    }

    serial.setBaudRate(QString(ui->baudrateLineEdit->text()).toInt());
    serial.setStopBits(QSerialPort::OneStop);
    serial.setParity(QSerialPort::NoParity);
    serial.setDataBits(QSerialPort::Data8);
    serial.setFlowControl(QSerialPort::NoFlowControl);
}

void LoggerWindow::pararCaptura()
{
    ui->capturarButton->setText("Capturar");
    ui->portLineEdit->setDisabled(false);
    ui->baudrateLineEdit->setDisabled(false);

    serial.close();
   // emit _pararCaptura();
}

void LoggerWindow::on_capturarButton_clicked()
{
    QString portS = ui->portLineEdit->text();
    QString baudrateS = ui->baudrateLineEdit->text();

    if(ui->capturarButton->text()=="Capturar")
    {
        capturar();

    }
    else
    {
        pararCaptura();

    }

    int baudrate;
    baudrate = baudrateS.toInt();

    qDebug() << portS << " " << baudrate;

}

void LoggerWindow::on_guardarButton_clicked()
{
    qDebug()<<"Vaaamos a guardar el fichero nomaas\n";
    QFile fitxer(ui->fitxerLogLineEdit->text());
    if (!fitxer.open(QIODevice::WriteOnly))
    {
        QMessageBox error;
        error.setIcon(QMessageBox::Critical);
        error.setText("ERROR: impossible crear el fitxer");
        error.exec();
    }
    else
    {
        QTextStream out(&fitxer);
        out << ui->monitorTextEdit->toPlainText();
        out.flush();
        fitxer.close();
    }

}
void LoggerWindow::datosEnBuffer()
{
    emit actualizarMonitor(serial.readAll());
}

void LoggerWindow::actualizarMonitor(QByteArray datos)
{

   QString nuevo = QString::fromStdString(datos.toStdString());

   QString res = "";
   if(filtroActivado)
   {
       if(sinPerdidas)
       {
           acumulado += nuevo;
           int offset;
           if((offset=filtro.indexIn(acumulado))!=-1)
           {
               res = filtro.cap(0);
               acumulado = acumulado.mid(offset+res.length());
           }
       }
       else if(filtro.indexIn(nuevo)!=-1)
       {
           res = filtro.cap(0);
       }
   }
   else
   {
        res = nuevo;
   }
   ui->nLiniesLineEdit->setText(
               QString::number(ui->nLiniesLineEdit->text().toInt()
                               +res.count('\n'))
               );
   textCursor.insertText(res);
   ui->monitorTextEdit->moveCursor (QTextCursor::End);
   emit nuevoMensaje(res);
}

void LoggerWindow::errorCritic(QString s)
{
    QMessageBox error;
    error.setIcon(QMessageBox::Critical);
    error.setText(s);
    error.exec();
}

void LoggerWindow::on_netejarButton_clicked()
{
    ui->monitorTextEdit->clear();
    ui->nLiniesLineEdit->setText("0");
}

void LoggerWindow::on_fileBrowserButton_clicked()
{
    QString dirname = QFileDialog::getExistingDirectory(this,tr("Open Image"), QDir::homePath());
    ui->fitxerLogLineEdit->setText(dirname);

    QString logfilename;
    bool ok = false;
    do{

        logfilename = QInputDialog::getText(this, tr("Nom del fitxer de log"),
                                                   tr("Fitxer:"), QLineEdit::Normal, "",&ok);
        ok = ok && logfilename.length() > 0;

    } while (!ok);
    ui->fitxerLogLineEdit->setText(dirname+"/"+logfilename);
}

void LoggerWindow::comprobarFiltro()
{
    filtroActivado = ui->filtreLineEdit->text().length() > 0;
//    ui->sinPerdidasCheckBox->setCheckable(filtroActivado);
    ui->sinPerdidasCheckBox->setEnabled(filtroActivado);
    if(filtroActivado)
        filtro.setPattern(ui->filtreLineEdit->text());

}

void LoggerWindow::on_filtreLineEdit_textChanged(const QString &arg1)
{
    qDebug() << "Filtre editat: " <<arg1;
    acumulado = "";
    comprobarFiltro();
}

void LoggerWindow::on_pushButton_clicked()
{
    QUrl url("http://doc.qt.io/qt-4.8/qregexp.html#details");
    QDesktopServices::openUrl(url);

}

void LoggerWindow::on_tcpCheckBox_toggled(bool checked)
{
    ui->tcpPortLineEdit->setEnabled(!checked);
    if(checked)
    {
        //Crear servidor TCP en port especificat.
        int port;
        QRegExp reg("^\\d+$");
        QString sp = ui->tcpPortLineEdit->text();
        if(reg.indexIn(sp) != -1)
            port = sp.toInt();
        else
            port = 8001;

        qDebug() << port;

        ui->tcpPortLineEdit->setText(QString::number(port));
        tcpServer.listen(QHostAddress::LocalHost,port);
    }
    else
    {
        tcpServer.close();
        emit tcpServer.closed();
    }
}
