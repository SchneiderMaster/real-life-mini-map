cmake_minimum_required(VERSION 3.10)
project(OsmiumReader)

set(CMAKE_CXX_STANDARD 17)

# Standard-Includepfade
include_directories(/usr/include)

# Pakete finden
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(EXPAT REQUIRED)
find_package(BZip2 REQUIRED)
find_package(X11 REQUIRED)

# PkgConfig nutzen, um Imlib2 (unter Arch/Hyprland meist Standard) zu finden
find_package(PkgConfig REQUIRED)
pkg_check_modules(IMLIB2 REQUIRED imlib2)

# Osmium Setup
set(PROTOZERO_INCLUDE_DIR "./include")
include_directories("${PROTOZERO_INCLUDE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS pbf xml io)

if(NOT OSMIUM_FOUND)
    message(WARNING "Libosmium not found!\n")
endif()

# Include-Verzeichnisse setzen
include_directories(
    ${OSMIUM_INCLUDE_DIRS}
    ${IMLIB2_INCLUDE_DIRS}
)

# Executable definieren
add_executable(osmium_test main.cpp)

# Libraries linken
target_link_libraries(osmium_test 
    Threads::Threads 
    X11::X11
    ${ZLIB_LIBRARIES} 
    ${EXPAT_LIBRARIES} 
    ${BZIP2_LIBRARIES}
    ${IMLIB2_LIBRARIES}
)

# Linker-Flags für Imlib2 (Pfade zu den Libs)
link_directories(${IMLIB2_LIBRARY_DIRS})