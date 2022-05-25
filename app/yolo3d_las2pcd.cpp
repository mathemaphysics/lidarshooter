#include <iostream>
#include <memory>
#include <filesystem>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/Options.hpp>
#include <pdal/io/EptReader.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/geometry/triangle_mesh.h>

#include <tiledb/array.h>

using namespace std;

int main()
{
  filesystem::path input_path = filesystem::path("/workspaces/pointcloudtools/data/NEONDSSampleLiDARPointCloud.las");
  filesystem::path output_path = input_path.parent_path() / (input_path.stem().string() + "-binary.pcd");
  if (!filesystem::exists(input_path))
  {
    cerr << "File " << input_path.string() << " doesn't exist." << std::endl;
    return -1;
  }

  try
  {
    pdal::Option lasOpt("filename", input_path.string());
    pdal::Options lasOpts;
    lasOpts.add(lasOpt);
    pdal::PointTable table;
    pdal::LasReader lasReader;
    lasReader.setOptions(lasOpts);
    lasReader.prepare(table);
    pdal::PointViewSet pointViewSet = lasReader.execute(table);
    pdal::PointViewPtr pointView = *pointViewSet.begin();
    pdal::Dimension::IdList dims = pointView->dims();
    pdal::LasHeader lasHeader = lasReader.header();

    lasHeader.srs().dump();

    long t;
    double x, y, z, xs, ys, zs;
    int npoints = pointView->size();
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Get the scaling factors from the header
    xs = lasHeader.scaleX();
    ys = lasHeader.scaleY();
    zs = lasHeader.scaleZ();

    // Get the range of the points for later use
    long maxx = lasHeader.maxX();
    long maxy = lasHeader.maxY();
    long maxz = lasHeader.maxZ();
    long minx = lasHeader.minX();
    long miny = lasHeader.minY();
    long minz = lasHeader.minZ();
    cloud.width = npoints;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    pdal::PointId idx = 0;
    for (auto& point: cloud)
    {
      t = pointView->getFieldAs<long>(pdal::Dimension::Id::GpsTime, idx);
      x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, idx) * xs;
      y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, idx) * ys;
      z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, idx) * zs;

      // Put the LAS point in the cloud
      point.x = x;
      point.y = y;
      point.z = z;

      ++idx;
    }

    pcl::io::savePCDFileBinary(output_path.string(), cloud);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  

}
