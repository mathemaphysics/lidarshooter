#include "sensorsdialog.h"
#include "ui_sensorsdialog.h"

#include "mainwindow.h"

#include <iostream>

RowPushButton::RowPushButton(int __row, QString _label)
    : QPushButton(_label)
{
    // Store your location in the table
    _row = __row;

    // Connect actual button click to custom clicked with row
    connect(this, SIGNAL(clicked(void)), this, SLOT(rowButtonClicked(void)));
}

void RowPushButton::rowButtonClicked()
{
    // Emit our position in the row
    emit clickedRow(_row);
}

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
    sensorItemsModel->setColumnCount(4);
    sensorItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Device")));
    sensorItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));
    sensorItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("Start")));
    sensorItemsModel->setHeaderData(3, Qt::Orientation::Horizontal, QVariant(QString("Delete")));

    // Set up the meshes rows
    meshItemsModel->setColumnCount(3);
    meshItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Name")));
    meshItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));
    meshItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("Delete")));

    // Make connections
    //connect(ui->tableViewSensors, SIGNAL(clicked(QModelIndex)), parent, SLOT(slotTableClickedStopMeshProjector(QModelIndex)));
}

SensorsDialog::~SensorsDialog()
{
    delete ui;
    delete sensorItemsModel;
    delete meshItemsModel;
}

void SensorsDialog::setSensorRow(int _row, std::string _device, std::string _path)
{
    // Set the fields
    sensorItemsModel->setItem(_row, 0, new QStandardItem(QString(_device.c_str())));
    sensorItemsModel->setItem(_row, 1, new QStandardItem(QString(_path.c_str())));

    // Add the start/stop button
    auto startSensorButton = new RowPushButton(_row, "Start");
    auto deleteSensorButton = new RowPushButton(_row, "Delete");
    ui->tableViewSensors->setIndexWidget(sensorItemsModel->index(0, 2), startSensorButton);
    ui->tableViewSensors->setIndexWidget(sensorItemsModel->index(0, 3), deleteSensorButton);
    
    // Link them to their actions
    connect(startSensorButton, SIGNAL(clickedRow(int)), this, SLOT(deleteSensorRow(int)));
    connect(deleteSensorButton, SIGNAL(clickedRow(int)), this, SLOT(deleteSensorRow(int)));
}

void SensorsDialog::deleteSensorRow(int _row)
{
    delete ui->tableViewSensors->indexWidget(sensorItemsModel->index(_row, 2));
    delete ui->tableViewSensors->indexWidget(sensorItemsModel->index(_row, 3));
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
