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

    RTCRayHit4 rayhit4;
    rayhit4.ray.org_x[0] = 0.f; rayhit4.ray.org_y[0] = 0.f; rayhit4.ray.org_z[0] = -1.f;
    rayhit4.ray.org_x[1] = 0.f; rayhit4.ray.org_y[1] = 0.f; rayhit4.ray.org_z[1] = -1.f;
    rayhit4.ray.org_x[2] = 0.f; rayhit4.ray.org_y[2] = 0.f; rayhit4.ray.org_z[2] = -1.f;
    rayhit4.ray.org_x[3] = 0.f; rayhit4.ray.org_y[3] = 0.f; rayhit4.ray.org_z[3] = -1.f;

    rayhit4.ray.dir_x[0]  = 0.437870544204244; rayhit4.ray.dir_y[0] = 0.5819343209147206; rayhit4.ray.dir_z[0] = 0.6852895976591227;
    rayhit4.ray.dir_x[1]  = 0.18499438632552934; rayhit4.ray.dir_y[1] = 0.2753895209183792; rayhit4.ray.dir_z[1] = 0.9433650877557355;
    rayhit4.ray.dir_x[2]  = 0.18814417f; rayhit4.ray.dir_y[2] = 0.28221626f; rayhit4.ray.dir_z[2] = 0.94072087f;
    rayhit4.ray.dir_x[3]  = 0.08581487653093794; rayhit4.ray.dir_y[3] = 0.5085431814428848; rayhit4.ray.dir_z[3] = 0.8567494613794217;

    rayhit4.ray.tnear[0] = 0.f;
    rayhit4.ray.tnear[1] = 0.f;
    rayhit4.ray.tnear[2] = 0.f;
    rayhit4.ray.tnear[3] = 0.f;

    rayhit4.ray.tfar[0] = std::numeric_limits<float>::infinity();
    rayhit4.ray.tfar[1] = std::numeric_limits<float>::infinity();
    rayhit4.ray.tfar[2] = std::numeric_limits<float>::infinity();
    rayhit4.ray.tfar[3] = std::numeric_limits<float>::infinity();

    rayhit4.hit.geomID[0] = RTC_INVALID_GEOMETRY_ID;
    rayhit4.hit.geomID[1] = RTC_INVALID_GEOMETRY_ID;
    rayhit4.hit.geomID[2] = RTC_INVALID_GEOMETRY_ID;
    rayhit4.hit.geomID[3] = RTC_INVALID_GEOMETRY_ID;

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

    // Ray mask; -1 means valid, 0 means invalid, i.e. don't compute
    int valid[4] = {-1, -1, -1, -1};
    rtcIntersect4(valid, scene, &context, &rayhit4);

    for (int i = 0; i < 4; ++i)
    {
        if (rayhit4.hit.geomID[i] != RTC_INVALID_GEOMETRY_ID) {
            std::cout << "Intersection at t = " << rayhit4.ray.tfar[i] << std::endl;
        } else {
            std::cout << "No Intersection" << std::endl;
        }
    }

    std::cout << "Device query: " << rtcGetDeviceProperty(device, RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED) << std::endl;

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
