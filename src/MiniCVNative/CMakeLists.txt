cmake_minimum_required(VERSION 3.9)
include("cmake/Aardvark.cmake")

project(MiniCVNative)

# set(Ceres_DIR ${CMAKE_LIBRARY_PATH}../../.vcpkg/vcpkg/packages/ceres_x64-osx-release)
find_package(OpenCV CONFIG REQUIRED)

set(CMAKE_BUILD_TYPE, "Release")
if(UNIX)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffunction-sections -fdata-sections -fvisibility=hidden")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffunction-sections -fdata-sections -fvisibility=hidden")
endif()



set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(APPLE)
    set(CMAKE_SHARED_LINKER_FLAGS "-O3 -flto=thin -dead_strip")
elseif(UNIX)
    set(CMAKE_SHARED_LINKER_FLAGS "-O3 -Wl,--gc-sections -Wl,--strip-all")
endif()


add_definitions(-D_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR )

add_library(MiniCVNative SHARED MiniCVNative.h MiniCVNative.cpp ap3p.h ap3p.cpp aruco_nano.h fivepoint.h fivepoint.cpp)

# include_directories(~/vcpkg/packages/ceres_arm64-osx/include)
target_link_libraries(MiniCVNative PRIVATE opencv_ml libprotobuf opencv_core opencv_videoio opencv_calib3d opencv_features2d opencv_objdetect)

SET(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../libs/Native/MiniCV/") 
install(TARGETS MiniCVNative DESTINATION ${OS}/${ARCH})