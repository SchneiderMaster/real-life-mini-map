cmake_minimum_required(VERSION 3.10)
project(OsmiumReader)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/include)

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(EXPAT REQUIRED)
find_package(BZip2 REQUIRED)
find_package(X11 REQUIRED)

set(PROTOZERO_INCLUDE_DIR "/home/fynn/real-life-mini-map/include")
include_directories("${PROTOZERO_INCLUDE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS pbf xml io)

if(NOT OSMIUM_FOUND)
    message(WARNING "Libosmium not found!\n")
endif()

include_directories(${OSMIUM_INCLUDE_DIRS})

add_executable(osmium_test main.cpp)

target_link_libraries(osmium_test 
    Threads::Threads 
    X11::X11
    ${ZLIB_LIBRARIES} 
    ${EXPAT_LIBRARIES} 
    ${BZIP2_LIBRARIES}
)
