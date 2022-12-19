#ifndef SENSORSDIALOG_H
#define SENSORSDIALOG_H

#include <QDialog>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QPushButton>

namespace Ui {
class SensorsDialog;
class RowPushButton;
}

class RowPushButton : public QPushButton
{
    Q_OBJECT

public:
    RowPushButton(int _row, QString _label);
    ~RowPushButton() = default;

signals:
    void clickedRow(int);

private slots:
    void rowButtonClicked();

private:
    int _row;
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
    void setSensorRow(int _row, std::string _device, std::string _path);
    void deleteSensorRow(int _row);
    void setMeshRow(int _row, std::string _name, std::string _path);
    void deleteMeshRow(int _row);

private:
    Ui::SensorsDialog *ui;
    QStandardItemModel* sensorItemsModel;
    QStandardItemModel* meshItemsModel;
};

#endif // SENSORSDIALOG_H
