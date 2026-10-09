#pragma once
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QMutex>
#include <QDebug>

// 分级日志系统 — 同时输出到 qDebug + 文件
class Logger {
public:
    enum Level { Debug = 0, Info = 1, Warning = 2, Error = 3 };

    static Logger& instance() {
        static Logger s;
        return s;
    }

    void setLogFile(const QString& path) {
        QMutexLocker lock(&m_mutex);
        if (m_file.isOpen()) m_file.close();
        m_file.setFileName(path);
        m_file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
        m_stream.setDevice(&m_file);
    }

    void log(Level lvl, const QString& msg) {
        QMutexLocker lock(&m_mutex);
        static const char* levelStr[] = {"DBG", "INF", "WRN", "ERR"};
        QString line = QString("[%1] %2 %3")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
            .arg(levelStr[lvl])
            .arg(msg);

        // 控制台输出
        switch (lvl) {
        case Debug:   qDebug().noquote() << line; break;
        case Info:    qDebug().noquote() << line; break;
        case Warning: qWarning().noquote() << line; break;
        case Error:   qCritical().noquote() << line; break;
        }

        // 文件输出
        if (m_file.isOpen()) {
            m_stream << line << "\n";
            m_stream.flush();
        }
    }

    // 便捷接口 — 为后续替换需要 MainWindow* 的调用点设计
    static void debug(const QString& msg)   { instance().log(Debug, msg); }
    static void info(const QString& msg)    { instance().log(Info, msg); }
    static void warning(const QString& msg) { instance().log(Warning, msg); }
    static void error(const QString& msg)   { instance().log(Error, msg); }

private:
    Logger() = default;
    QMutex m_mutex;
    QFile m_file;
    QTextStream m_stream;
};
