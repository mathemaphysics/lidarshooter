add_executable(kdtree-example kdtree-example.cpp)
target_link_libraries(kdtree-example ${PCL_LIBRARIES} ${PDAL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tiledb-example tiledb-example.cpp)
target_link_libraries(tiledb-example ${PCL_LIBRARIES} ${PDAL_LIBRARIES} ${OpenCV_LIBRARIES} TileDB::tiledb_shared)

add_executable(wxapp-example wxapp-example.cpp)
target_link_libraries(wxapp-example ${wxWidgets_LIBRARIES})
