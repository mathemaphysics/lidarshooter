add_executable(yolo3d_app yolo3d_app.cpp)
target_link_libraries(yolo3d_app PRIVATE yolo3d)

add_executable(yolo3d_las2pcd yolo3d_las2pcd.cpp)
target_link_libraries(yolo3d_las2pcd PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(yolo3d_rasterize yolo3d_rasterize.cpp)
target_link_libraries(yolo3d_rasterize PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(yolo3d_ground yolo3d_ground.cpp)
target_link_libraries(yolo3d_ground PRIVATE yolo3d ${PDAL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Boost::system)
