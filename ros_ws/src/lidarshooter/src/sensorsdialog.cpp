#include "sensorsdialog.h"
#include "ui_sensorsdialog.h"

#include "mainwindow.h"

#include <iostream>

TaggedPushButton::TaggedPushButton(QString __tag, QString __label)
    : QPushButton(__label)
{
    // Store your location in the table
    _tag = __tag;

    // Connect actual button click to custom clicked with row
    connect(this, SIGNAL(clicked(void)), this, SLOT(rowButtonClicked(void)));
}

void TaggedPushButton::rowButtonClicked()
{
    // Emit our position in the row
    emit clickedRow(_tag);
}

SensorsDialog::SensorsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SensorsDialog)
{
    ui->setupUi(this);

    _sensorItemsModel = new QStandardItemModel(this);
    _meshItemsModel = new QStandardItemModel(this);
    ui->tableViewSensors->setModel(_sensorItemsModel);
    ui->tableViewMeshes->setModel(_meshItemsModel);

    // Set up the sensors rows
    _sensorItemsModel->setColumnCount(5);
    _sensorItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Device")));
    _sensorItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));
    _sensorItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("Start")));
    _sensorItemsModel->setHeaderData(3, Qt::Orientation::Horizontal, QVariant(QString("Stop")));
    _sensorItemsModel->setHeaderData(4, Qt::Orientation::Horizontal, QVariant(QString("Delete")));

    // Set up the meshes rows
    _meshItemsModel->setColumnCount(3);
    _meshItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Name")));
    _meshItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));
    _meshItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("Delete")));
}

SensorsDialog::~SensorsDialog()
{
    delete ui;

    // Clean up leftover button we new'ed in the addSensorRow calls
    auto parentWindow = dynamic_cast<MainWindow*>(parentWidget());
    for (auto [key, val] : parentWindow->deviceConfigMap)
        deleteSensorRow(QString(key.c_str())); // Only deletes anything if it exists

    delete _sensorItemsModel;
    delete _meshItemsModel;
}

void SensorsDialog::addSensorRow(std::string _device, std::string _path)
{
    // Append a row then get the new index of that row
    QList<QStandardItem*> thisRow;
    thisRow.append(new QStandardItem(QString(_device.c_str())));
    thisRow.append(new QStandardItem(QString(_path.c_str())));

    // Make sure the item isn't adding a duplicate
    auto itemsList = _sensorItemsModel->findItems(_device.c_str(), Qt::MatchExactly, 0); // Match sensorUid column
    if (itemsList.length() > 0)
        return;

    // Otherwise it isn't present so add it
    _sensorItemsModel->appendRow(thisRow);
    auto itemIndex = _sensorItemsModel->indexFromItem(thisRow.at(0)); // Device string should be unique
    
    // Add the start/stop button
    auto startSensorButton = new TaggedPushButton(_device.c_str(), "Start");
    auto stopSensorButton = new TaggedPushButton(_device.c_str(), "Stop");
    auto deleteSensorButton = new TaggedPushButton(_device.c_str(), "Delete");
    ui->tableViewSensors->setIndexWidget(_sensorItemsModel->index(itemIndex.row(), 2), startSensorButton);
    ui->tableViewSensors->setIndexWidget(_sensorItemsModel->index(itemIndex.row(), 3), stopSensorButton);
    ui->tableViewSensors->setIndexWidget(_sensorItemsModel->index(itemIndex.row(), 4), deleteSensorButton);

    // Link them to their actions
    connect(startSensorButton, SIGNAL(clickedRow(QString)), dynamic_cast<MainWindow*>(parentWidget()), SLOT(startMeshProjector(QString))); // FIXME: Do this when you have it all figured out
    connect(stopSensorButton, SIGNAL(clickedRow(QString)), dynamic_cast<MainWindow*>(parentWidget()), SLOT(stopMeshProjector(QString))); // FIXME: Do this when you have it all figured out
    connect(deleteSensorButton, SIGNAL(clickedRow(QString)), this, SLOT(deleteSensorRow(QString)));
}

int SensorsDialog::getSensorRow(QString _tag)
{
    auto itemList = _sensorItemsModel->findItems(_tag);
    if (itemList.length() > 0)
        return _sensorItemsModel->indexFromItem(itemList.at(0)).row(); // Just take the first one; unique
    else
        return -1; // No item found by that name b/c length <= 0
}

void SensorsDialog::deleteSensorRow(QString _tag)
{
    auto itemList = _sensorItemsModel->findItems(_tag);
    if (itemList.length() == 0)
        return; // Because we can't get itemIndex from itemList.at(0); it doesn't exist

    // Get the QModelIndex and free the individual buttons
    auto itemIndex = _sensorItemsModel->indexFromItem(itemList.at(0));
    delete ui->tableViewSensors->indexWidget(_sensorItemsModel->index(itemIndex.row(), 2));
    delete ui->tableViewSensors->indexWidget(_sensorItemsModel->index(itemIndex.row(), 3));
    delete ui->tableViewSensors->indexWidget(_sensorItemsModel->index(itemIndex.row(), 4));
    
    // Drop the row now that things memory is freed
    _sensorItemsModel->removeRow(itemIndex.row());
}

void SensorsDialog::setMeshRow(int _row, std::string _device, std::string _path)
{
    _meshItemsModel->setItem(0, 0, new QStandardItem(_device.c_str()));
    _meshItemsModel->setItem(0, 1, new QStandardItem(_path.c_str()));
}

void SensorsDialog::deleteMeshRow(int _row)
{
    _meshItemsModel->removeRow(_row);
}

std::string SensorsDialog::getSensorName(int _index)
{
    return _sensorItemsModel->data(_sensorItemsModel->index(_index, 0)).toString().toStdString();
}

std::string SensorsDialog::getMeshName(int _index)
{
    return _meshItemsModel->data(_meshItemsModel->index(_index, 0)).toString().toStdString();
}
