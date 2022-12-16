#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#include <spdlog/spdlog.h>

#include <memory>
#include <thread>
#include <typeinfo>
#include <utility>
#include <type_traits>

#include "XYZIRPoint.hpp"

namespace lidarshooter
{

class CloudConverter : public std::enable_shared_from_this<CloudConverter>
{
public:
    static std::shared_ptr<CloudConverter> create(pcl::PCLPointCloud2::ConstPtr __cloud);
    std::shared_ptr<CloudConverter> getPtr();
    void setCloud(pcl::PCLPointCloud2::ConstPtr __cloud);

    template <class C = lidarshooter::XYZIRPoint, class P = pcl::PointXYZ>
    void to(std::shared_ptr<pcl::PointCloud<P>> _output)
    {
        if (std::is_same<P, pcl::PointXYZ>::value)
        {
            // Convert to PointCloud<PointXYZ>
            _output->clear();
            _output->width = _cloud->width;
            _output->height = _cloud->height;
            _output->is_dense = false;
            _output->resize(_output->width * _output->height);
        
            // TODO: This should be threaded
            unsigned int numTotalPoints = _cloud->width * _cloud->height;
            unsigned int numThreads = 4; // TODO: Make this a parameter
            unsigned int startPointIndex = 0;

            // Overall: for (std::size_t jdx = 0; jdx < numTotalPoint; ++jdx)
            std::vector<std::thread> threads;
            for (int threadIdx = 0; threadIdx < numThreads; ++threadIdx)
            {
                // Number of iterations for the current threadIdx
                unsigned int numIterations =
                    numTotalPoints / numThreads
                        + (threadIdx < numTotalPoints % numThreads ? 1 : 0);

                // Start thread index threadIdx
                threads.emplace_back(
                    [this, _output, numIterations, threadIdx, startPointIndex]() {
                        for (std::size_t jdx = startPointIndex; jdx < startPointIndex + numIterations; ++jdx)
                        {
                            // No mutex required since we're accessing different locations
                            auto rawData = _cloud->data.data() + jdx * _cloud->point_step;

                            // TODO: Generalize this to what will likely be different point types
                            // TODO: Generate an extractor class directly from the "fields" field of PointCloud2
                            float px, py, pz;
                            auto point = C(const_cast<std::uint8_t*>(rawData));
                            point.getPoint(&px, &py, &pz, nullptr, nullptr);

                            // Linear position update here
                            _output->points[jdx].x = px;
                            _output->points[jdx].y = py;
                            _output->points[jdx].z = pz;
                        }
                    }
                );

                // Each block might be different size
                startPointIndex += numIterations;
            }

            // For the sake of sanity block here
            for (auto threadItr = threads.begin(); threadItr != threads.end(); ++threadItr)
                threadItr->join();
        }
        else if (std::is_same<P, pcl::PointCloud<pcl::PointXYZRGB>>::value)
        {
            // Convert to PointCloud<PointXYZRGB>
        }
    }
private:
    CloudConverter(pcl::PCLPointCloud2::ConstPtr __cloud);
    pcl::PCLPointCloud2::ConstPtr _cloud;
};

}