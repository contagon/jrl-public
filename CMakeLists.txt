cmake_minimum_required(VERSION 3.15...3.31)
project(jrl VERSION 1.1.1 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
find_package(GTSAM 4.2.0 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# BUILD: JRL Library
file(GLOB_RECURSE jrl_srcs "src/*.cpp")
file(GLOB_RECURSE jrl_headers "include/*.h")
add_library(jrl ${jrl_srcs} ${jrl_headers})
set_target_properties(jrl PROPERTIES PUBLIC_HEADER "${jrl_headers}")
target_link_libraries(jrl gtsam nlohmann_json::nlohmann_json)
target_include_directories(jrl PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)

# Build Tests for JRL if configured
option(JRL_BUILD_TESTS "Build unit tests" OFF)
if (${JRL_BUILD_TESTS})
    add_subdirectory(tests)
endif()

# Build Python Bindings for JRL if configured (built by default when
# building via scikit-build-core)
option(JRL_BUILD_PYTHON "Build python bindings" OFF)
if (${JRL_BUILD_PYTHON} OR SKBUILD)
    add_subdirectory(python)
endif()

# INSTALL: JRL Library
# Skipped when building via scikit-build-core, which only wants the wheel
if(NOT SKBUILD)
    install(
        TARGETS jrl
        EXPORT jrl-exports
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/jrl)
    include(cmake/HandleUninstall.cmake) # for "make jrl-uninstall"

    # Setup configuration for find_package
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MakeConfigFile.cmake)
    MakeConfigFile(jrl)
    export(EXPORT jrl-exports FILE ${CMAKE_CURRENT_BINARY_DIR}/jrl-exports.cmake)
endif()
