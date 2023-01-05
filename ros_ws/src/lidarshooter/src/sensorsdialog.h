#ifndef SENSORSDIALOG_H
#define SENSORSDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QPushButton>
#include <QString>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/qt_sinks.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <memory>

// Forward declaration of parent class
class MainWindow;

namespace Ui {
class SensorsDialog;
class TaggedPushButton;
}

class TaggedPushButton : public QPushButton
{
    Q_OBJECT

public:
    TaggedPushButton(QString __tag, QString __label);
    ~TaggedPushButton() = default;

signals:
    void clickedRow(QString);

private slots:
    void rowButtonClicked();

private:
    QString _tag;
};

class SensorsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SensorsDialog(QWidget *parent = nullptr);
    ~SensorsDialog();
    
    std::string getSensorName(int _index);
    std::string getMeshName(int _index);

signals:

public slots:
    int getSensorRow(QString _tag);
    void addSensorRow(std::string _device, std::string _path);
    void deleteSensorRow(QString _tag);
    void setMeshRow(int _row, std::string _name, std::string _path);
    void deleteMeshRow(int _row);

private:
    Ui::SensorsDialog *ui;
    QStandardItemModel* _sensorItemsModel;
    QStandardItemModel* _meshItemsModel;
};

#endif // SENSORSDIALOG_H
