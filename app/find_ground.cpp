#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <filesystem>
#include <boost/program_options.hpp>

int main(int argc, char** argv)
{
    // Handle command line input
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
      ("help", "Display help")
      ("input", boost::program_options::value<std::string>(), "Point file to load")
    ;

    // Parse and store command line variables for later
    boost::program_options::variables_map variables;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), variables);
    boost::program_options::notify(variables);

    // Stop if help requested
    if (variables.count("help") > 0)
    {
      std::cout << desc << std::endl;
      return 0;
    }

    std::filesystem::path inputFile;
    if (variables.count("input") == 1)
    {
        inputFile = variables["input"].as<std::string>();
        if (!std::filesystem::exists(inputFile))
        {
            std::cerr << "Input points file " << inputFile << " does not exist" << std::endl;
            return -1;
        }
    }
    else
    {
        std::cerr << "Either no or too many input files gives" << std::endl;
        return -1;
    }

    //// Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Object for storing both the points and the normals.
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);

    // Read a PCD file from disk.
    pcl::io::loadPCDFile<pcl::PointXYZ>(inputFile.string(), *cloud);

    // Normal estimation.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(1.0);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // The triangulation object requires the points and normals to be stored in the same structure.
    pcl::concatenateFields(*cloud, *normals, *cloudNormals);
    // Tree object for searches in this new object.
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointNormal>);
    kdtree2->setInputCloud(cloudNormals);

    // Triangulation object.
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> triangulation;
    // Output object, containing the mesh.
    pcl::PolygonMesh triangles, moreTriangles;
    // Maximum distance between connected points (maximum edge length).
    triangulation.setSearchRadius(2.0);
    // Maximum acceptable distance for a point to be considered,
    // relative to the distance of the nearest point.
    triangulation.setMu(4.0);
    // How many neighbors are searched for.
    triangulation.setMaximumNearestNeighbors(10);
    // Points will not be connected to the current point
    // if their normals deviate more than the specified angle.
    triangulation.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees.
    // If false, the direction of normals will not be taken into account
    // when computing the angle between them.
    triangulation.setNormalConsistency(false);
    // Minimum and maximum angle there can be in a triangle.
    // The first is not guaranteed, the second is.
    triangulation.setMinimumAngle(M_PI / 18); // 10 degrees.
    triangulation.setMaximumAngle(2 * M_PI / 3); // 120 degrees.

    // Triangulate the cloud.
    triangulation.setInputCloud(cloudNormals);
    triangulation.setSearchMethod(kdtree2);
    triangulation.reconstruct(triangles);

    // Save to disk.
    pcl::io::saveVTKFile("bunny-mesh.vtk", triangles);
    pcl::io::loadPolygonFileSTL("/workspaces/data/boxcar.stl", moreTriangles);
    std::cout << moreTriangles << std::endl;
}
