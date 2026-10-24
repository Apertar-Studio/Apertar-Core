cmake_minimum_required(VERSION 3.20)

project(ApertarCore
    VERSION 0.1.0
    DESCRIPTION "Cinema-first libcamera backend for Apertar UI"
    LANGUAGES CXX
)

# ApertarCore is a real-time capture application. A CMake tree configured
# without a build type otherwise gets an unoptimized encoder on single-config
# generators, which is not fast enough for high-resolution recording.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

option(APERTARCORE_ENABLE_LIBCAMERA "Build with direct libcamera support when available" ON)
option(APERTARCORE_ENABLE_LIBAV "Build lazy FFmpeg/libav H.264 and ProRes encoder plugins when available" ON)
set(APERTARCORE_PRORES_FFMPEG_PREFIX "" CACHE PATH "Optional private optimized FFmpeg prefix for ProRes only")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED IMPORTED_TARGET yaml-0.1)
pkg_check_modules(ALSA REQUIRED alsa)
find_package(JPEG QUIET)
find_path(TURBOJPEG_INCLUDE_DIR turbojpeg.h)
find_library(TURBOJPEG_LIBRARY NAMES turbojpeg libturbojpeg)

if(APERTARCORE_ENABLE_LIBCAMERA OR APERTARCORE_ENABLE_LIBAV)
    find_package(PkgConfig QUIET)
endif()

if(APERTARCORE_ENABLE_LIBCAMERA)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBCAMERA QUIET IMPORTED_TARGET libcamera libcamera-base)
    endif()
endif()

if(APERTARCORE_ENABLE_LIBAV)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBAV QUIET IMPORTED_TARGET libavcodec libavformat libavutil)
        if(APERTARCORE_PRORES_FFMPEG_PREFIX)
            # FindPkgConfig otherwise reuses the previous prefix's headers/libraries.
            if(NOT APERTARCORE_PRORES_FFMPEG_PREFIX STREQUAL _APERTAR_PRORES_CONFIGURED_PREFIX)
                get_cmake_property(_apertar_cache_vars CACHE_VARIABLES)
                foreach(_apertar_cache_var IN LISTS _apertar_cache_vars)
                    if(_apertar_cache_var MATCHES "^(PRORES_LIBAV_|pkgcfg_lib_PRORES_LIBAV_|__pkg_config_(checked|arguments)_PRORES_LIBAV$)")
                        unset(${_apertar_cache_var} CACHE)
                    endif()
                endforeach()
                set(_APERTAR_PRORES_CONFIGURED_PREFIX "${APERTARCORE_PRORES_FFMPEG_PREFIX}" CACHE INTERNAL "Last resolved ProRes prefix" FORCE)
            endif()
            # Do not let the private ProRes codec replace H.264/system FFmpeg.
            set(_apertar_pkg_path "$ENV{PKG_CONFIG_PATH}")
            set(_apertar_pkg_libdir "$ENV{PKG_CONFIG_LIBDIR}")
            set(ENV{PKG_CONFIG_PATH} "")
            set(ENV{PKG_CONFIG_LIBDIR} "${APERTARCORE_PRORES_FFMPEG_PREFIX}/lib/pkgconfig")
            pkg_check_modules(PRORES_LIBAV REQUIRED IMPORTED_TARGET libavcodec>=62 libavformat>=62 libavutil>=60)
            set(ENV{PKG_CONFIG_PATH} "${_apertar_pkg_path}")
            set(ENV{PKG_CONFIG_LIBDIR} "${_apertar_pkg_libdir}")
        endif()
    endif()
endif()

add_executable(apertar-core
    src/main.cpp
    src/ApertarCoreApp.cpp
    src/audio/AudioRecordingSession.cpp
    src/audio/TimestampedAudioCapture.cpp
    src/camera/CameraController.cpp
    src/cdng/CdngEncoder.cpp
    src/cdng/LosslessJpegEncoder.cpp
    src/color/ColorProfileConfig.cpp
    src/color/ProresIspProfile.cpp
    src/lossydng/LossyDngEncoder.cpp
    src/prores/ProresRecorder.cpp
    src/mlv/MlvEncoder.cpp
    src/video/LibavH264EncoderClient.cpp
    src/hardware/GpioInput.cpp
    src/hardware/StandbyPowerButtonInput.cpp
    src/hardware/HardwareScriptRunner.cpp
    src/hardware/GroveAnalogInput.cpp
    src/hardware/Mpu6050RecordingSession.cpp
    src/hardware/NeoRotary4Input.cpp
    src/hardware/Pcf8574Input.cpp
    src/hardware/RecordingIndicatorOutput.cpp
    src/hardware/TriggerOutput.cpp
    src/ipc/ControlProtocol.cpp
    src/ipc/ControlServer.cpp
    src/web/WebControlPage.cpp
    src/web/WebServer.cpp
)

set(APERTARCORE_PRIVATE_INCLUDE_DIRS
    include
    src
)

if(TURBOJPEG_INCLUDE_DIR)
    list(APPEND APERTARCORE_PRIVATE_INCLUDE_DIRS ${TURBOJPEG_INCLUDE_DIR})
endif()

target_include_directories(apertar-core
    PRIVATE
        ${APERTARCORE_PRIVATE_INCLUDE_DIRS}
        ${ALSA_INCLUDE_DIRS}
)

target_link_libraries(apertar-core
    PRIVATE
        Threads::Threads
        PkgConfig::YAML
        ${CMAKE_DL_LIBS}
)

if(TURBOJPEG_INCLUDE_DIR AND TURBOJPEG_LIBRARY)
    target_link_libraries(apertar-core PRIVATE ${TURBOJPEG_LIBRARY})
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_TURBOJPEG=1)
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_JPEG=1)
elseif(JPEG_FOUND)
    target_link_libraries(apertar-core PRIVATE JPEG::JPEG)
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_TURBOJPEG=0)
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_JPEG=1)
else()
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_TURBOJPEG=0)
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_JPEG=0)
endif()

if(LIBCAMERA_FOUND)
    target_link_libraries(apertar-core PRIVATE PkgConfig::LIBCAMERA)
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_LIBCAMERA=1)
else()
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_LIBCAMERA=0)
endif()

if(LIBAV_FOUND)
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_LIBAV=1)

    add_library(apertar-h264-libav MODULE
        src/video/LibavH264EncoderPlugin.cpp
    )
    target_include_directories(apertar-h264-libav PRIVATE src)
    target_link_libraries(apertar-h264-libav PRIVATE PkgConfig::LIBAV)
    target_compile_options(apertar-h264-libav PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(apertar-h264-libav PROPERTIES PREFIX "")

    message(STATUS "ApertarCore H.264 libav backend: lazy plugin enabled")
    add_library(apertar-prores-libav MODULE
        src/prores/ProresPlugin.cpp
        src/prores/ProresEncoder.cpp
        src/prores/ProresColor.cpp
        src/prores/ProresRepair.cpp
    )
    target_include_directories(apertar-prores-libav PRIVATE src/prores include)
    if(PRORES_LIBAV_FOUND AND APERTARCORE_PRORES_FFMPEG_PREFIX)
        target_link_libraries(apertar-prores-libav PRIVATE PkgConfig::PRORES_LIBAV Threads::Threads)
        set_target_properties(apertar-prores-libav PROPERTIES
            BUILD_RPATH "${APERTARCORE_PRORES_FFMPEG_PREFIX}/lib"
            INSTALL_RPATH "${APERTARCORE_PRORES_FFMPEG_PREFIX}/lib")
        message(STATUS "ApertarCore ProRes private FFmpeg: ${APERTARCORE_PRORES_FFMPEG_PREFIX}")
    else()
        target_link_libraries(apertar-prores-libav PRIVATE PkgConfig::LIBAV Threads::Threads)
    endif()
    target_compile_options(apertar-prores-libav PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(apertar-prores-libav PROPERTIES PREFIX "")
else()
    target_compile_definitions(apertar-core PRIVATE APERTARCORE_HAS_LIBAV=0)
    message(STATUS "ApertarCore H.264 libav backend: disabled")
endif()

target_compile_options(apertar-core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

if(NOT CMAKE_CROSSCOMPILING)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-mtune=native" APERTARCORE_HAS_MTUNE_NATIVE)
    if(APERTARCORE_HAS_MTUNE_NATIVE)
        target_compile_options(apertar-core PRIVATE $<$<CONFIG:Release>:-mtune=native>)
    endif()
endif()

install(TARGETS apertar-core RUNTIME DESTINATION bin)
if(TARGET apertar-h264-libav)
    install(TARGETS apertar-h264-libav LIBRARY DESTINATION lib/apertar-core)
endif()
if(TARGET apertar-prores-libav)
    install(TARGETS apertar-prores-libav LIBRARY DESTINATION lib/apertar-core)
endif()
