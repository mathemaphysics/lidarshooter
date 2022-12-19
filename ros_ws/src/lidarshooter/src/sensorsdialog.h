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
    
    void setRow(int row, std::string device, std::string path);
    void deleteRow(int row);

private:
    Ui::SensorsDialog *ui;
    QStandardItemModel* sensorItemsModel;
};

#endif // SENSORSDIALOG_H
