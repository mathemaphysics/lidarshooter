add_executable(yolo3dapp yolo3dapp.cpp)
target_link_libraries(yolo3dapp PRIVATE yolo3d)

add_executable(las2pcd las2pcd.cpp)
target_link_libraries(las2pcd PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(rasterize rasterize.cpp)
target_link_libraries(rasterize PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(find_ground find_ground.cpp)
target_link_libraries(find_ground PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Boost::system Boost::program_options)

add_executable(testing_ground testing_ground.cpp)
target_link_libraries(find_ground PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Boost::system Boost::program_options)

add_executable(embree_test embree_test.cpp)
target_include_directories(embree_test PUBLIC /usr/src/embree-3.13.4.x86_64.linux/include)
target_link_directories(embree_test PUBLIC /usr/src/embree-3.13.4.x86_64.linux/lib)
target_link_libraries(embree_test PRIVATE embree3 Boost::system Boost::program_options ${PCL_LIBRARIES} Boost::system Boost::program_options)
