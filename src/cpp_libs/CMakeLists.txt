

add_library(mylib2_pcl SHARED
    my_pcl/pcl_commons.cpp
    my_pcl/pcl_io.cpp
    my_pcl/pcl_visualization.cpp
    my_pcl/pcl_filters.cpp
    my_pcl/pcl_advanced.cpp
)

add_library(mylib2_basics SHARED
    my_basics/basics.cpp
    my_basics/eigen_funcs.cpp
)

target_link_libraries( mylib2_basics
    ${THIRD_PARTY_LIBS} 
)


target_link_libraries( mylib2_pcl
    ${THIRD_PARTY_LIBS} 
    mylib2_basics
)

# ----------------------- scripts -----------------------

add_library(detect_object_from_cloud SHARED
    detect_object_from_cloud.cpp
)
target_link_libraries( detect_object_from_cloud
    ${THIRD_PARTY_LIBS} 
    mylib2_basics mylib2_pcl
    ${catkin_LIBRARIES} 
)





