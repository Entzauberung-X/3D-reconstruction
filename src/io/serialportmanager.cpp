#include "io/serialportmanager.h"
#include <QDebug>

SerialPortManager::SerialPortManager(QObject *parent) : QObject(parent), m_serialPort(nullptr)
{
    m_serialPort = new QSerialPort(this);
    
    // 绑定底层串口的接收信号到我们的处理函数
    connect(m_serialPort, &QSerialPort::readyRead, this, &SerialPortManager::handleReadyRead);
    // 绑定错误信号
    connect(m_serialPort, &QSerialPort::errorOccurred, this, &SerialPortManager::handleError);
}

SerialPortManager::~SerialPortManager()
{
    closePort();
}

QStringList SerialPortManager::getAvailablePorts()
{
    QStringList ports;
    // 自动扫描当前虚拟机识别到的所有串口设备
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        ports << info.portName();
    }
    return ports;
}

bool SerialPortManager::openPort(const QString &portName, qint32 baudRate)
{
    if (m_serialPort->isOpen()) {
        m_serialPort->close();
    }

    // 配置串口参数 (必须与STM32端完全一致：115200, 8位数据位, 无校验, 1位停止位)
    m_serialPort->setPortName(portName);
    m_serialPort->setBaudRate(baudRate);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (m_serialPort->open(QIODevice::ReadWrite)) {
        qDebug() << "串口打开成功:" << portName;
        return true;
    } else {
        qDebug() << "串口打开失败:" << m_serialPort->errorString();
        emit errorOccurred(m_serialPort->errorString());
        return false;
    }
}

void SerialPortManager::closePort()
{
    if (m_serialPort && m_serialPort->isOpen()) {
        m_serialPort->close();
        qDebug() << "串口已关闭";
    }
}

void SerialPortManager::sendData(const QByteArray &data)
{
    if (!m_serialPort || !m_serialPort->isOpen()) {
        emit errorOccurred("串口未打开，发送失败！");
        return;
    }
    m_serialPort->write(data);
}

bool SerialPortManager::isOpen() const
{
    return m_serialPort && m_serialPort->isOpen();
}

void SerialPortManager::handleReadyRead()
{
    // 读取串口缓冲区中的所有数据
    QByteArray data = m_serialPort->readAll();
    if (!data.isEmpty()) {
        // 发射信号，把数据送出去 (不在这里处理数据，遵循解耦原则)
        emit dataReceived(data);
    }
}

void SerialPortManager::handleError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError && error != QSerialPort::TimeoutError) {
        emit errorOccurred(m_serialPort->errorString());
    }
}
