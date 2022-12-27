#include "cloudupdater.h"

CloudUpdater::CloudUpdater()
{

}

void CloudUpdater::start(MainWindow* mainWindow, QString _sensorUid)
{
    auto sensorUid = _sensorUid.toStdString();
    while (true)
    {
        // The "external use" mutex being used when checking whether cloud updated
        while (mainWindow->meshProjectorMap[sensorUid]->cloudWasUpdated() == false)
        {
            // Check to see if we're being shut down
            if (mainWindow->traceThreadInitMap[sensorUid].load() == false)
                break;

            // Otherwise wait to avoid CPU pinning
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Might be a flaw to wait until first trace to init
        }

        // Need the break here too to guarantee no waiting time at shutdown
        if (mainWindow->traceThreadInitMap[sensorUid].load() == false)
            break;

        // Update the cloud in the viewer
        mainWindow->updateTraceInViewer(sensorUid);

        // Add some padding to guarantee no CPU pinning
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}