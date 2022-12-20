#ifndef SENSORSDIALOG_H
#define SENSORSDIALOG_H

#include <QDialog>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QPushButton>
#include <QString>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/qt_sinks.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <memory>

namespace Ui {
class SensorsDialog;
class TaggedPushButton;
}

class TaggedPushButton : public QPushButton
{
    Q_OBJECT

public:
    TaggedPushButton(QString __tag, QString __label, std::shared_ptr<spdlog::logger> __logger = nullptr);
    ~TaggedPushButton() = default;

signals:
    void clickedRow(QString);

private slots:
    void rowButtonClicked();

private:
    QString _tag;
    std::shared_ptr<spdlog::logger> _logger;

    const std::string _className = "TaggedPushButton";
};

class SensorsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SensorsDialog(QWidget *parent = nullptr, std::shared_ptr<spdlog::logger> __logger = nullptr);
    ~SensorsDialog();
    
    std::string getSensorName(int _index);
    std::string getMeshName(int _index);

signals:

public slots:
    void addSensorRow(std::string _device, std::string _path);
    void deleteSensorRow(QString _tag);
    void setMeshRow(int _row, std::string _name, std::string _path);
    void deleteMeshRow(int _row);

private:
    Ui::SensorsDialog *ui;
    QStandardItemModel* _sensorItemsModel;
    QStandardItemModel* _meshItemsModel;
    std::shared_ptr<spdlog::logger> _logger;

    const std::string _className = "SensorsDialog";
};

#endif // SENSORSDIALOG_H
