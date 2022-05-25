#include <iostream>
#include <memory>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/Options.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/flann.h>
#include <pcl/surface/mls.h>
#include <openni2/OpenNI.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem/path.hpp>

using namespace std;

int main()
{
  boost::filesystem::path input_path = boost::filesystem::path("/workspaces/data/las/08_18_Left_A_CL360006.las");
  boost::filesystem::path output_path = input_path.parent_path() / (input_path.stem().string() + ".pcd");
  //if (!boost::filesystem::exists(input_path))
  //{
  //  cerr << "File " << input_path.string() << " doesn't exist.";
  //  return -1;
  //}

  try
  {
    openni::OpenNI::initialize();
    openni::Device device;
    openni::VideoStream depth;
    openni::VideoFrameRef frame;
    openni::Status state;
    state = device.open(openni::ANY_DEVICE);
    cout << "Open: State: " << state << endl;
    state = depth.create(device, openni::SENSOR_DEPTH);
    cout << "Create: State: " << state << endl;
    depth.start();
    depth.readFrame(&frame);
    int width = frame.getWidth();
    int height = frame.getHeight();
    cout << "Width:  " << width << endl;
    cout << "Height: " << height << endl;
    cv::Mat depthData(height, width, CV_16UC1, const_cast<void*>(frame.getData()));
    cout << "Value: " << depthData.at<unsigned short>(200, 200) << endl;
    cv::imwrite("depth_image.jpg", depthData);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  return 0;
}
