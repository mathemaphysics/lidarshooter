add_executable(yolo3d yolo3d.cpp)
target_link_libraries(yolo3d PRIVATE yolo3d)

add_executable(las2pcd las2pcd.cpp)
target_link_libraries(las2pcd PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(rasterize rasterize.cpp)
target_link_libraries(rasterize PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(find_ground find_ground.cpp)
target_link_libraries(find_ground PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Boost::system)
