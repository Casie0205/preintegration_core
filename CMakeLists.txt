cmake_minimum_required(VERSION 3.16)

project(preintegration_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(PREINTEGRATION_CORE_BUILD_TESTS "Build preintegration_core tests" ON)
option(PREINTEGRATION_CORE_BUILD_EXAMPLES "Build preintegration_core examples" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(preintegration_core
    src/preintegrator.cpp
)
add_library(preintegration_core::preintegration_core ALIAS preintegration_core)

target_compile_features(preintegration_core PUBLIC cxx_std_17)
target_include_directories(preintegration_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(preintegration_core PUBLIC Eigen3::Eigen)

if(MSVC)
    target_compile_options(preintegration_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(preintegration_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(PREINTEGRATION_CORE_BUILD_TESTS)
    enable_testing()
    add_executable(test_preintegration_core tests/test_preintegration_core.cpp)
    target_link_libraries(test_preintegration_core PRIVATE preintegration_core::preintegration_core)
    add_test(NAME preintegration_core_tests COMMAND test_preintegration_core)
endif()

if(PREINTEGRATION_CORE_BUILD_EXAMPLES)
    add_executable(example_preintegration examples/simple_integration.cpp)
    target_link_libraries(example_preintegration PRIVATE preintegration_core::preintegration_core)
endif()

install(TARGETS preintegration_core
    EXPORT preintegration_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT preintegration_coreTargets
    FILE preintegration_coreTargets.cmake
    NAMESPACE preintegration_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/preintegration_core
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/preintegration_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/preintegration_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/preintegration_core
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/preintegration_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/preintegration_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/preintegration_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/preintegration_core
)
