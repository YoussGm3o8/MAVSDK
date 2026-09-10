cmake_minimum_required(VERSION 3.13)

project(external-nlohmann_json)
include(ExternalProject)

list(APPEND CMAKE_ARGS
    "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}"
    "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
    "-DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}"
    "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
    "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
    "-DBUILD_SHARED_LIBS=OFF"
    "-DJSON_BuildTests=OFF"
    "-DJSON_Install=ON"
    )

if(IOS)
    list(APPEND CMAKE_ARGS
        "-DPLATFORM=${PLATFORM}"
        "-DDEPLOYMENT_TARGET=${DEPLOYMENT_TARGET}"
        "-DENABLE_STRICT_TRY_COMPILE=${ENABLE_STRICT_TRY_COMPILE}"
        )
endif()

# DOWNLOAD_EXTRACT_TIMESTAMP was added in CMake 3.24. Older CMake versions
# already preserve archive timestamps, so only pass the option when supported.
set(DOWNLOAD_OPTIONS)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    list(APPEND DOWNLOAD_OPTIONS DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
endif()

message(STATUS "Preparing external project \"nlohmann_json\" with args:")
foreach(CMAKE_ARG ${CMAKE_ARGS})
    message(STATUS "-- ${CMAKE_ARG}")
endforeach()

ExternalProject_add(
    nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz
    URL_HASH SHA256=4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187
    ${DOWNLOAD_OPTIONS}
    PREFIX nlohmann_json
    CMAKE_ARGS "${CMAKE_ARGS}"
    )
