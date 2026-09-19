find_package(nlohmann_json REQUIRED)

add_executable(unit_tests_runner
    ${UNIT_TEST_SOURCES}
)

target_compile_definitions(unit_tests_runner PRIVATE FAKE_TIME=1 TESTING=1 ASIO_NO_EXCEPTIONS=1)

set_target_properties(unit_tests_runner
    PROPERTIES COMPILE_FLAGS ${warnings}
)

find_package(GTest REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(fmt REQUIRED)

target_link_libraries(unit_tests_runner
    mavsdk
    nlohmann_json::nlohmann_json
    tinyxml2::tinyxml2
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
    fmt::fmt
)

if(WIN32 AND BUILD_SHARED_LIBS)
    add_custom_command(TARGET unit_tests_runner POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:mavsdk> $<TARGET_FILE_DIR:unit_tests_runner>)
endif()

target_include_directories(unit_tests_runner
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../mavsdk/core
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../mavsdk/core
)


add_test(NAME unit_tests
    COMMAND unit_tests_runner
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
