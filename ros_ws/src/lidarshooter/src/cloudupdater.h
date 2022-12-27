#ifndef CLOUDUPDATER_H
#define CLOUDUPDATER_H

#include <QObject>
#include <QOpenGLContext>
#include <QThread>

#include "mainwindow.h"

#include <chrono>
#include <thread>

class CloudUpdater : public QObject
{
    Q_OBJECT

public:
    CloudUpdater();
    ~CloudUpdater() = default;

public slots:
    void start(MainWindow*, QString);

signals:
    void ready();

private:
    
};

#endif // CLOUDUPDATER_H
