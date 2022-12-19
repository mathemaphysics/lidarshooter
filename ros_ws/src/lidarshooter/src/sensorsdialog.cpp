#include "sensorsdialog.h"
#include "ui_sensorsdialog.h"

SensorsDialog::SensorsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SensorsDialog)
{
    ui->setupUi(this);
    sensorItemsModel = new QStandardItemModel(this);
    ui->tableViewSensors->setModel(sensorItemsModel);
    
    // Set up the sensors rows
    sensorItemsModel->setColumnCount(2);
    sensorItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Device")));
    sensorItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));

    // Make connections
    connect(ui->tableViewSensors, SIGNAL(clicked(QModelIndex)), parent, SLOT(slotTableClickedStopMeshProjector(QModelIndex)));
}

SensorsDialog::~SensorsDialog()
{
    delete ui;
}

void SensorsDialog::setRow(int _row, std::string _device, std::string _path)
{
    sensorItemsModel->setItem(0, 0, new QStandardItem(QString(_device.c_str())));
    sensorItemsModel->setItem(0, 1, new QStandardItem(QString(_path.c_str())));
}

void SensorsDialog::deleteRow(int _row)
{
    sensorItemsModel->removeRow(_row);
}
