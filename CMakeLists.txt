cmake_minimum_required(VERSION 3.20)

project(ApertarCore
    VERSION 0.1.0
    DESCRIPTION "Cinema-first libcamera backend for Apertar UI"
    LANGUAGES CXX
)

option(APERTARCORE_ENABLE_LIBCAMERA "Build with direct libcamera support when available" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

if(APERTARCORE_ENABLE_LIBCAMERA)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBCAMERA QUIET IMPORTED_TARGET libcamera libcamera-base)
    endif()
endif()

add_executable(apertar-core
    src/main.cpp
    src/ApertarCoreApp.cpp
    src/camera/CameraController.cpp
    src/cdng/CdngEncoder.cpp
    src/ipc/ControlProtocol.cpp
    src/ipc/ControlServer.cpp
)

target_include_directories(apertar-core
    PRIVATE
        include
        src
)

target_link_libraries(apertar-core
    PRIVATE
        Threads::Threads
)

if(LIBCAMERA_FOUND)
    target_link_libraries(apertar-core PRIVATE PkgConfig::LIBCAMERA)
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_LIBCAMERA=1)
else()
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_LIBCAMERA=0)
endif()

target_compile_options(apertar-core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

install(TARGETS apertar-core RUNTIME DESTINATION bin)
