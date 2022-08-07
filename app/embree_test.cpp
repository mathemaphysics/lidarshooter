// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <embree3/rtcore.h>
#include <filesystem>
#include <limits>
#include <iostream>
#include <boost/program_options.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

float getIntersection()
{
    RTCDevice device = rtcNewDevice(NULL);
    RTCScene scene   = rtcNewScene(device);
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    float* vb = (float*) rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), 4);
    vb[0] = 0.f; vb[1] = 0.f; vb[2] = 0.f; // 1st vertex
    vb[3] = 1.f; vb[4] = 0.f; vb[5] = 0.f; // 2nd vertex
    vb[6] = 0.f; vb[7] = 1.f; vb[8] = 0.f; // 3rd vertex
    vb[9] = 0.f; vb[10] = 0.f; vb[11] = -0.3f; // 3rd vertex

    unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned), 2);
    ib[0] = 0; ib[1] = 1; ib[2] = 2;
    ib[3] = 3; ib[4] = 1; ib[5] = 2;

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    RTCRayHit rayhit; 
    rayhit.ray.org_x  = 0.f; rayhit.ray.org_y = 0.f; rayhit.ray.org_z = -1.f;
    rayhit.ray.dir_x  = 0.18814417f; rayhit.ray.dir_y = 0.28221626f; rayhit.ray.dir_z = 0.94072087f;
    rayhit.ray.tnear  = 0.f;
    rayhit.ray.tfar   = std::numeric_limits<float>::infinity();
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    rtcIntersect1(scene, &context, &rayhit);

    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        std::cout << "Intersection at t = " << rayhit.ray.tfar << std::endl;
    } else {
        std::cout << "No Intersection" << std::endl;
    }

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return 0;
}

int main(int argc, char **argv)
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

    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(inputFile.string(), mesh);

    std::cout << getIntersection() << std::endl;

    return 0;
}
