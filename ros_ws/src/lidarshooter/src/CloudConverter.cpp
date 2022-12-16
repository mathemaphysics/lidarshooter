#include "CloudConverter.hpp"

lidarshooter::CloudConverter::CloudConverter(pcl::PCLPointCloud2::ConstPtr __cloud)
{
    _cloud = __cloud;
}

std::shared_ptr<lidarshooter::CloudConverter> lidarshooter::CloudConverter::create(pcl::PCLPointCloud2::ConstPtr _cloud)
{
    return std::shared_ptr<lidarshooter::CloudConverter>(new lidarshooter::CloudConverter(_cloud));
}

std::shared_ptr<lidarshooter::CloudConverter> lidarshooter::CloudConverter::getPtr()
{
    return shared_from_this();
}

void lidarshooter::CloudConverter::setCloud(pcl::PCLPointCloud2::ConstPtr __cloud)
{
    _cloud = __cloud;
}
