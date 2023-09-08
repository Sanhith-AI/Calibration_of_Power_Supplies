#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QList>
#include <QComboBox>
#include <QString>
#include <QTextEdit>
#include <QTime>
#include <QDebug>
#include <QtSerialPort/QSerialPortInfo>
#include <QtSerialPort>
#include <QFile>
#include <QList>

QString line ;
  QString txt3;
QSerialPort *serial;
//QSerialPort *serial1;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    serialbuffer="";
    ui->lcdNumber->display("------------");
    ui->lcdNumber_2->display("------------");
    ui->lcdNumber_3->display("------------");
    QList<QSerialPortInfo> list;
        list = QSerialPortInfo::availablePorts();

        for (int i= 0; i < list.length(); i++)
        {
           ui->comBox->addItem(list[i].portName());

        }
        ui->baudBox->addItem(QStringLiteral("1200"), QSerialPort::Baud1200);
        ui->baudBox->addItem(QStringLiteral("2400"), QSerialPort::Baud2400);
        ui->baudBox->addItem(QStringLiteral("4800"), QSerialPort::Baud4800);
        ui->baudBox->addItem(QStringLiteral("9600"), QSerialPort::Baud9600);
        ui->baudBox->addItem(QStringLiteral("38400"), QSerialPort::Baud38400);
        ui->baudBox->addItem(QStringLiteral("57600"), QSerialPort::Baud57600);
        ui->baudBox->addItem(QStringLiteral("115200"), QSerialPort::Baud115200);

        ui->comboBox->addItem("K1(1A)", QVariant(1));
        ui->comboBox->addItem("K2(100mA)", QVariant(2));
        ui->comboBox->addItem("K3(10mA)", QVariant(3));
        ui->comboBox->addItem("K4(1mA)", QVariant(4));
        ui->comboBox->addItem("K5(100µA)", QVariant(5));
        ui->comboBox->addItem("K6", QVariant(6));
        ui->comboBox->addItem("K7", QVariant(7));

        ui->comboBox_2->addItem("no averaging", QVariant(0));
        ui->comboBox_2->addItem("10", QVariant(1));
        ui->comboBox_2->addItem("50", QVariant(2));
        ui->comboBox_2->addItem("100", QVariant(3));
        ui->comboBox_2->addItem("200", QVariant(4));
        ui->comboBox_2->addItem("500", QVariant(5));
        ui->comboBox_2->addItem("1000", QVariant(6));
        ui->comboBox_2->addItem("2000", QVariant(7));


}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{
        serial = new QSerialPort (this) ;

        serial->setPortName(ui->comBox->currentText());
        serial->setBaudRate (ui->baudBox->currentData().toInt());
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity (QSerialPort::NoParity);
        serial->setStopBits (QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        serial->open(QIODevice::ReadWrite);
        connect(serial,SIGNAL(readyRead()),this, SLOT (serialReceived()));

}


void MainWindow::on_pushButton_2_clicked()
{
    if (serial->isOpen())
    {
              serial->close();
    }
   /* if(serial1->isOpen())
    {
    serial1->close();
    }*/
     qDebug()<<"serialport closed succesfully";
     //ui->seak1->setText("serial port closed. click on connect to receive data again..");
     ui->seak1_2->setText("serial port closed. click on connect to receive data again..");
}

void MainWindow::serialReceived()
{


    QStringList buffer_split = serialbuffer.split(",");

     if(buffer_split.length() < 3){

      serialdata=serial->readAll();
      serialbuffer = serialbuffer + QString::fromStdString(serialdata.toStdString());
      serialdata.clear();
     }

     else{
             // the second element of buffer_split is parsed correctly, update the temperature value on temp_lcdNumber

             qDebug() << buffer_split;
             serialbuffer = "";
            // int num1 = buffer_split[2].toInt();
             ui->lcdNumber->display(buffer_split[0]);
             ui->lcdNumber_2->display(buffer_split[1]);
             ui->lcdNumber_3->display(buffer_split[2]);
     }

     // ui->receiver->append("\n" + serialbuffer + "\n");

     // ui->seak->setText("Data received successfully");
      ui->seak_2->setText("Voltage received successfully");
      ui->seak_3->setText("Current received successfully");
      ui->seak_4->setText("Temperature received successfully");

 }






void MainWindow::on_pushButton_3_clicked()
{

     if (serial->isOpen())
     {
           QString testOuts;
           QString testOuts1;


         testOuts=ui->comboBox->currentData().toInt();

       testOuts1=ui->comboBox_2->currentData().toInt();

         serial->write((char*)testOuts.data(),testOuts.length());

        serial->write((char*)testOuts1.data(),testOuts1.length());

     qDebug()<<"message sent";
    // ui->seak1->setText("message sent successfully. Wait few seconds to get back the response");
     ui->seak1_2->setText("message sent successfully. Wait few seconds to get back the response");

     }
    /* else if(serial1->isOpen())
     {
         QString value;
         value=ui->comboBox->currentData().toInt();
         QString value1;
         value=ui->comboBox_2->currentData().toInt();
         serial->write((char*)value.data(),value.length());
         serial->write((char*)value1.data(),value.length());
        qDebug()<<"message sent";
        //ui->seak1->setText("message sent successfully. Wait few seconds to see the response...");
        ui->seak1_2->setText("message sent successfully. Wait few seconds to see the response...");
     }*/

     else
     {
         qDebug()<<"message not sent";
         //ui->seak1->setText("message not sent.Check serial port open or not");
         ui->seak1_2->setText("message not sent.Check serial port open or not");
     }
}


/*void MainWindow::on_pushButton_4_clicked()
{
    if(serial->isOpen())
    {
        serial->close();
    }
    serial1 = new QSerialPort (this) ;
    serial1->setPortName(ui->comBox->currentText());
    serial1->setBaudRate (ui->baudBox->currentData().toInt());
    serial1->setDataBits(QSerialPort::Data8);
    serial1->setParity (QSerialPort::NoParity);
    serial1->setStopBits (QSerialPort::OneStop);
    serial1->setFlowControl(QSerialPort::NoFlowControl);
    serial1->open(QIODevice::ReadWrite);
    connect(serial1,SIGNAL(readyRead()),this, SLOT (serialReceived1()));

}*/

/*void MainWindow::serialReceived1()
{
QString receivedData;
    receivedData.append(serial1->readLine());


        bool ok;
        QString binaryNumber = QString("%1").arg(receivedData.toULongLong(&ok, 16),8, 2, QChar('0'));
        qDebug()<<"received data:"<<binaryNumber;//to see data in console
        ui->bina->append("\n" + binaryNumber + "\n");
        ui->seak->setText("Data Received successfully -->" "\n");



    receivedData.clear();
}*/





