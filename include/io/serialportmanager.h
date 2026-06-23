#ifndef SERIALPORTMANAGER_H
#define SERIALPORTMANAGER_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <QByteArray>

class SerialPortManager : public QObject
{
    Q_OBJECT
    // 如果你使用的是较老的Qt版本（<5.15），请将上面那行替换为传统的 Q_OBJECT 宏声明方式：
    // Q_OBJECT
public:
    explicit SerialPortManager(QObject *parent = nullptr);
    ~SerialPortManager();

    // 获取当前电脑可用的串口名列表 (如 "COM3", "/dev/ttyUSB0")
    QStringList getAvailablePorts();

    // 打开串口 (portName: 串口名, baudRate: 波特率，默认115200，需与STM32一致)
    bool openPort(const QString &portName, qint32 baudRate = 115200);

    // 关闭串口
    void closePort();

    // 发送数据 (支持发送字符串或十六进制数组)
    void sendData(const QByteArray &data);

    // 检查串口是否已打开
    bool isOpen() const;

signals:
    // 当收到STM32发来的数据时，触发此信号，将数据传递给外部
    void dataReceived(const QByteArray &data);

    // 串口错误信号（用于在UI上提示）
    void errorOccurred(const QString &errorString);

private slots:
    // 内部槽函数：响应底层串口的 readyRead 信号
    void handleReadyRead();

    // 内部槽函数：响应底层串口的错误信号
    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort *m_serialPort;
};

#endif // SERIALPORTMANAGER_H
