cmake_minimum_required(VERSION 3.0.0)
project(robo_v3 VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(robo_v3 main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

# rplidar library here
add_library(rplidar STATIC IMPORTED)
set_property(TARGET rplidar
  PROPERTY IMPORTED_LOCATION /home/nano/Desktop/lidar/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a)
target_include_directories(rplidar INTERFACE
/home/nano/Desktop/lidar/rplidar_sdk/sdk/include/
/home/nano/Desktop/lidar/rplidar_sdk/sdk/src/
/home/nano/Desktop/lidar/rplidar_sdk/obj/Linux/Release/sdk/src/arch/linux)

target_link_libraries(${PROJECT_NAME} rplidar)



