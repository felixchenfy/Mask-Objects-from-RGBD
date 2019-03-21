
add_subdirectory( cpp_libs )

add_executable(node_detect_object_from_cloud nodes/node_detect_object_from_cloud.cpp )
target_link_libraries( node_detect_object_from_cloud
    mylib2_pcl mylib2_basics detect_object_from_cloud
    ${catkin_LIBRARIES} 
)
