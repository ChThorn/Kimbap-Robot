#ifndef LOGGER_H
#define LOGGER_H

#include <QFile>
#include <QObject>
#include <QTextStream>
#include <QDateTime>


class Logger:QObject
{
    Q_OBJECT

    static Logger instance;
public:
    // Constructor
    Logger();

    void init();
    void write(const QString &str, bool print=true);
    static void write_space();
private:
    QFile *file;
    QString filename;
    QString getFileName();
};

#endif // LOGGER_H
