#include "sensorsdialog.h"
#include "ui_sensorsdialog.h"

SensorsDialog::SensorsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SensorsDialog)
{
    ui->setupUi(this);
    sensorItemsModel = new QStandardItemModel(this);
    meshItemsModel = new QStandardItemModel(this);
    ui->tableViewSensors->setModel(sensorItemsModel);
    ui->tableViewMeshes->setModel(meshItemsModel);
    
    // Set up the sensors rows
    sensorItemsModel->setColumnCount(2);
    sensorItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Device")));
    sensorItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));
    sensorItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("")));
    sensorItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("")));

    // Set up the meshes rows
    meshItemsModel->setColumnCount(2);
    meshItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Name")));
    meshItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));

    // Make connections
    connect(ui->tableViewSensors, SIGNAL(clicked(QModelIndex)), parent, SLOT(slotTableClickedStopMeshProjector(QModelIndex)));
}

SensorsDialog::~SensorsDialog()
{
    delete ui;
    delete sensorItemsModel;
    delete meshItemsModel;
}

void SensorsDialog::setSensorRow(int _row, std::string _device, std::string _path)
{
    sensorItemsModel->setItem(0, 0, new QStandardItem(QString(_device.c_str())));
    sensorItemsModel->setItem(0, 1, new QStandardItem(QString(_path.c_str())));
}

void SensorsDialog::deleteSensorRow(int _row)
{
    sensorItemsModel->removeRow(_row);
}

void SensorsDialog::setMeshRow(int _row, std::string _device, std::string _path)
{
    meshItemsModel->setItem(0, 0, new QStandardItem(QString(_device.c_str())));
    meshItemsModel->setItem(0, 1, new QStandardItem(QString(_path.c_str())));
}

void SensorsDialog::deleteMeshRow(int _row)
{
    meshItemsModel->removeRow(_row);
}

std::string SensorsDialog::getSensorName(int _index)
{
    return sensorItemsModel->data(sensorItemsModel->index(_index, 0)).toString().toStdString();
}

std::string SensorsDialog::getMeshName(int _index)
{
    return meshItemsModel->data(meshItemsModel->index(_index, 0)).toString().toStdString();
}
