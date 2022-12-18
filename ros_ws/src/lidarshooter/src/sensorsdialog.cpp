#include "sensorsdialog.h"
#include "ui_sensorsdialog.h"

SensorsDialog::SensorsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SensorsDialog)
{
    ui->setupUi(this);
}

SensorsDialog::~SensorsDialog()
{
    delete ui;
}
