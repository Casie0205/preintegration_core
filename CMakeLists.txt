cmake_minimum_required(VERSION 3.16)

project(preintegration_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(PREINTEGRATION_CORE_BUILD_TESTS "Build preintegration_core tests" ON)
option(PREINTEGRATION_CORE_BUILD_EXAMPLES "Build preintegration_core examples" ON)
option(
    PREINTEGRATION_CORE_BUILD_GTSAM_REGRESSION
    "Build the optional GTSAM cross-regression test"
    OFF
)
option(
    PREINTEGRATION_CORE_BUILD_OB_GINS_REGRESSION
    "Build the optional OB_GINS cross-regression test"
    OFF
)
set(
    PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR
    ""
    CACHE PATH
    "Path to the OB_GINS source tree for external regression"
)

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

    add_executable(
        test_preintegration_reference
        tests/test_preintegration_reference.cpp
    )
    target_link_libraries(
        test_preintegration_reference
        PRIVATE
            preintegration_core::preintegration_core
    )
    add_test(
        NAME preintegration_core_reference_tests
        COMMAND test_preintegration_reference
    )
endif()

if(PREINTEGRATION_CORE_BUILD_EXAMPLES)
    add_executable(example_preintegration examples/simple_integration.cpp)
    target_link_libraries(example_preintegration PRIVATE preintegration_core::preintegration_core)
endif()

if(PREINTEGRATION_CORE_BUILD_GTSAM_REGRESSION)
    find_package(GTSAM REQUIRED)
    add_executable(
        test_preintegration_gtsam
        tests/test_preintegration_gtsam.cpp
    )
    target_link_libraries(
        test_preintegration_gtsam
        PRIVATE
            preintegration_core::preintegration_core
            gtsam
    )
    get_filename_component(
        GTSAM_RUNTIME_LIBRARY_DIR
        "${GTSAM_DIR}/../.."
        ABSOLUTE
    )
    set_target_properties(
        test_preintegration_gtsam
        PROPERTIES
            BUILD_RPATH "${GTSAM_RUNTIME_LIBRARY_DIR}"
    )
    add_test(
        NAME preintegration_core_gtsam_regression
        COMMAND test_preintegration_gtsam
    )
    set_tests_properties(
        preintegration_core_gtsam_regression
        PROPERTIES
            ENVIRONMENT
                "LD_LIBRARY_PATH=${GTSAM_RUNTIME_LIBRARY_DIR}:$ENV{LD_LIBRARY_PATH}"
    )
endif()

if(PREINTEGRATION_CORE_BUILD_OB_GINS_REGRESSION)
    if(NOT IS_DIRECTORY "${PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR}")
        message(
            FATAL_ERROR
            "PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR must point to OB_GINS"
        )
    endif()

    add_library(
        ob_gins_preintegration_regression_support
        STATIC
        ${PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR}/src/preintegration/preintegration_base.cc
        ${PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR}/src/preintegration/preintegration_normal.cc
    )
    target_compile_features(
        ob_gins_preintegration_regression_support
        PUBLIC
            cxx_std_17
    )
    target_include_directories(
        ob_gins_preintegration_regression_support
        PUBLIC
            ${PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR}
            ${PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR}/src/preintegration
            ${PREINTEGRATION_CORE_OB_GINS_SOURCE_DIR}/src/thirdparty/abseil-cpp
    )
    target_link_libraries(
        ob_gins_preintegration_regression_support
        PUBLIC
            Eigen3::Eigen
    )

    find_package(glog CONFIG QUIET)
    if(TARGET glog::glog)
        target_link_libraries(
            ob_gins_preintegration_regression_support
            PUBLIC
                glog::glog
        )
    endif()

    add_executable(
        test_preintegration_ob_gins
        tests/test_preintegration_ob_gins.cpp
    )
    target_link_libraries(
        test_preintegration_ob_gins
        PRIVATE
            preintegration_core::preintegration_core
            ob_gins_preintegration_regression_support
    )
    add_test(
        NAME preintegration_core_ob_gins_regression
        COMMAND test_preintegration_ob_gins
    )
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
