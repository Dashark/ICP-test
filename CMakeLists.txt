cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project( icp_samples )
set(CMAKE_CXX_STANDARD 14 CACHE STRING "The target C++ standard. PCL requires C++14 or higher.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp1_simple src/icp1_simple.cpp)
target_link_libraries (icp1_simple ${PCL_LIBRARIES})

add_executable (icp2_iterative_view src/icp2_iterative_view.cpp)
target_link_libraries (icp2_iterative_view ${PCL_LIBRARIES})

add_executable (example_scope_time src/example_scope_time.cpp)
target_link_libraries (example_scope_time ${PCL_LIBRARIES})

add_executable (example_extract_indices src/example_extract_indices.cpp)
target_link_libraries (example_extract_indices ${PCL_LIBRARIES})

add_executable (example_remove_nan_from_point_cloud src/example_remove_nan_from_point_cloud.cpp)
target_link_libraries (example_remove_nan_from_point_cloud ${PCL_LIBRARIES})

add_executable (icp3_with_normal_iterative_view src/icp3_with_normal_iterative_view.cpp)
target_link_libraries (icp3_with_normal_iterative_view ${PCL_LIBRARIES})

add_executable (icp4_after_feature_registration src/icp4_after_feature_registration.cpp src/visualize_correspondences.cpp)
target_link_libraries (icp4_after_feature_registration ${PCL_LIBRARIES})


add_executable (transform_estimation src/transform_estimation.cpp)
target_link_libraries (transform_estimation ${PCL_LIBRARIES})

add_executable (passthrough_filter src/passthrough_filter.cpp)
target_link_libraries (passthrough_filter ${PCL_LIBRARIES})

add_executable (voxel_grid src/voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

add_executable (statistical_removal src/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (remove_outliers src/remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})