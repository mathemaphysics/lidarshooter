#ifndef SENSORSDIALOG_H
#define SENSORSDIALOG_H

#include <QDialog>

namespace Ui {
class SensorsDialog;
}

class SensorsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SensorsDialog(QWidget *parent = nullptr);
    ~SensorsDialog();

private:
    Ui::SensorsDialog *ui;
};

#endif // SENSORSDIALOG_H
