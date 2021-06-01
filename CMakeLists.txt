cmake_minimum_required(VERSION 3.1)
project(uwb_base)
add_compile_options(--std=c++17)

find_package(Boost 1.72 REQUIRED)
find_package(OpenMP)

include_directories(
    include
)

add_executable(${PROJECT_NAME} 
    src/main.cc
    src/base_station_comm.cc
)

target_link_libraries(${PROJECT_NAME} 
    -lpthread 
    -pthread
    OpenMP::OpenMP_CXX
)