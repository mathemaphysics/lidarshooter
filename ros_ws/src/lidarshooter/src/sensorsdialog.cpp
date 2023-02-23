#include "sensorsdialog.h"
#include "ui_sensorsdialog.h"

#include "mainwindow.h"

#include <iostream>

TaggedPushButton::TaggedPushButton(QString __tag, QString __label, QWidget* _parent)
    : QPushButton(__label, _parent)
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

TaggedCheckbox::TaggedCheckbox(QString __tag, QWidget* _parent)
    : QCheckBox(_parent)
{
    // Set the tag for reference
    _tag = __tag;

    // Make it default on
    setChecked(true);

    // Connect toggled to rowToggled slot to emit
    connect(this, SIGNAL(clicked(bool)), this, SLOT(emitRowToggled(bool)));
}

void TaggedCheckbox::emitRowToggled(bool _toggled)
{
    emit rowToggled(_tag, _toggled);
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
    _sensorItemsModel->setColumnCount(4);
    _sensorItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Device")));
    ui->tableViewSensors->setColumnWidth(0, 100); // Sensor UID
    _sensorItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));
    ui->tableViewSensors->setColumnWidth(1, 200); // Path to this UID's config file
    _sensorItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("Delete")));
    ui->tableViewSensors->setColumnWidth(2, 50); // Delete button for this item
    _sensorItemsModel->setHeaderData(3, Qt::Orientation::Horizontal, QVariant(QString("Publish")));
    ui->tableViewSensors->setColumnWidth(3, 50); // Delete button for this item

    // Set up the meshes rows
    _meshItemsModel->setColumnCount(3);
    _meshItemsModel->setHeaderData(0, Qt::Orientation::Horizontal, QVariant(QString("Name")));
    ui->tableViewMeshes->setColumnWidth(0, 100); // Sensor UID
    _meshItemsModel->setHeaderData(1, Qt::Orientation::Horizontal, QVariant(QString("Path")));
    ui->tableViewMeshes->setColumnWidth(1, 200); // Path to this UID's config file
    _meshItemsModel->setHeaderData(2, Qt::Orientation::Horizontal, QVariant(QString("Delete")));
    ui->tableViewMeshes->setColumnWidth(2, 50); // Delete button for this item
}

SensorsDialog::~SensorsDialog()
{
    // Clean up leftover button we new'ed in the addSensorRow calls
    auto parentWindow = dynamic_cast<MainWindow*>(parentWidget());
    for (auto [key, val] : parentWindow->deviceConfigMap)
        deleteSensorRow(QString(key.c_str())); // Only deletes anything if it exists

    delete ui;
    delete _sensorItemsModel;
    delete _meshItemsModel;
}

std::string SensorsDialog::getSensorName(int _index)
{
    return _sensorItemsModel->data(_sensorItemsModel->index(_index, 0)).toString().toStdString();
}

std::string SensorsDialog::getMeshName(int _index)
{
    return _meshItemsModel->data(_meshItemsModel->index(_index, 0)).toString().toStdString();
}

int SensorsDialog::getSensorRow(QString _tag)
{
    auto itemList = _sensorItemsModel->findItems(_tag);
    if (itemList.length() > 0)
        return _sensorItemsModel->indexFromItem(itemList.at(0)).row(); // Just take the first one; unique
    else
        return -1; // No item found by that name b/c length <= 0
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
    auto deleteSensorButton = new TaggedPushButton(_device.c_str(), "Delete", ui->tableViewSensors);
    ui->tableViewSensors->setIndexWidget(_sensorItemsModel->index(itemIndex.row(), 2), deleteSensorButton);

    // Add the publish checkbox
    auto publishCheckbox = new TaggedCheckbox(_device.c_str(), ui->tableViewSensors);
    ui->tableViewSensors->setIndexWidget(_sensorItemsModel->index(itemIndex.row(), 3), publishCheckbox);

    // Link them to their actions
    auto mainWindow = dynamic_cast<MainWindow*>(parentWidget());
    publishCheckbox->setChecked(false);

    // Don't construct DeviceRuntime directly with operator[] call
    auto runtimeIterator = mainWindow->runtimeMap.find(_device);
    if (runtimeIterator == mainWindow->runtimeMap.end())
        return;

    // Make sure it's set to false initially even though it should be
    runtimeIterator->second.setCloudPublishState(false);

    // TODO: If we make default DeviceRuntime to not publish, this isn't needed

    connect(deleteSensorButton, SIGNAL(clickedRow(QString)), mainWindow, SLOT(deleteSensor(QString))); // Have to do this first because
    connect(deleteSensorButton, SIGNAL(clickedRow(QString)), this, SLOT(deleteSensorRow(QString))); // This one knows the device key and needs to be here
    connect(publishCheckbox, SIGNAL(rowToggled(QString, bool)), mainWindow, SLOT(updatePublishCloud(QString, bool)));
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
    
    // Drop the row now that things memory is freed
    _sensorItemsModel->removeRow(itemIndex.row());
}

void SensorsDialog::setMeshRow(int _row, std::string _device, std::string _path)
{
    _meshItemsModel->setItem(0, 0, new QStandardItem(_device.c_str()));
    _meshItemsModel->setItem(0, 1, new QStandardItem(_path.c_str()));
}

void SensorsDialog::addMeshRow(std::string _meshName, std::string _path)
{
    // Append a row then get the new index of that row
    QList<QStandardItem*> thisRow;
    thisRow.append(new QStandardItem(QString(_meshName.c_str())));
    thisRow.append(new QStandardItem(QString(_path.c_str())));

    // Make sure the item isn't adding a duplicate
    auto itemsList = _meshItemsModel->findItems(_meshName.c_str(), Qt::MatchExactly, 0); // Match sensorUid column
    if (itemsList.length() > 0)
        return;

    // Now that we know it isn't already in there
    _meshItemsModel->appendRow(thisRow);
    auto itemIndex = _sensorItemsModel->indexFromItem(thisRow.at(0)); // Device string should be unique

    // Add the start/stop button
    auto deleteMeshButton = new TaggedPushButton(_meshName.c_str(), "Delete", ui->tableViewMeshes);
    ui->tableViewMeshes->setIndexWidget(_meshItemsModel->index(itemIndex.row(), 2), deleteMeshButton);
    
    // Link them to their actions
    auto mainWindow = dynamic_cast<MainWindow*>(parentWidget());
    connect(deleteMeshButton, SIGNAL(clickedRow(QString)), mainWindow, SLOT(deleteMesh(QString))); // Have to do this first because
    connect(deleteMeshButton, SIGNAL(clickedRow(QString)), this, SLOT(deleteMeshRow(QString))); // This one knows the device key and needs to be here
}

void SensorsDialog::deleteMeshRow(QString _tag)
{
    auto itemList = _meshItemsModel->findItems(_tag);
    if (itemList.length() == 0)
        return; // Because we can't get itemIndex from itemList.at(0); it doesn't exist

    // Get the QModelIndex and free the individual buttons
    auto itemIndex = _meshItemsModel->indexFromItem(itemList.at(0));
    delete ui->tableViewMeshes->indexWidget(_meshItemsModel->index(itemIndex.row(), 2));
    
    // Drop the row now that things memory is freed
    _meshItemsModel->removeRow(itemIndex.row());
}

void SensorsDialog::emitSensorToggled(QString _sensorUid, bool _toggled)
{
    emit sensorToggled(_sensorUid, _toggled);
}
