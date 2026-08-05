# nanobind provides the NB_MODULE macros and build helper. `nanobind` is
# sourced by scikit-build-core's build requirements.
include(FetchContent)
find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module OPTIONAL_COMPONENTS Development.SABIModule)

FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind
    GIT_TAG v2.13.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(nanobind)

# Python Bindings
nanobind_add_module(_core bindings.cpp)
target_link_libraries(_core PRIVATE jrl gtsam nlohmann_json::nlohmann_json)

install(TARGETS _core DESTINATION jrl)
