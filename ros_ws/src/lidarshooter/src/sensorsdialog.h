#ifndef SENSORSDIALOG_H
#define SENSORSDIALOG_H

#include <QDialog>
#include <QStandardItem>
#include <QStandardItemModel>

namespace Ui {
class SensorsDialog;
}

class SensorsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SensorsDialog(QWidget *parent = nullptr);
    ~SensorsDialog();
    
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
