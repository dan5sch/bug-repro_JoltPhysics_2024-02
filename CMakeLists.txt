cmake_minimum_required(VERSION 3.14)

project(JoltBugRepro)

# Language / tooling requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compile settings
add_compile_options(-W -Wall)

# External dependencies
find_package(Threads REQUIRED)
include(FetchContent)
option(INTERPROCEDURAL_OPTIMIZATION "Enable interprocedural optimizations" OFF)
FetchContent_Declare(
    JoltPhysics
    SOURCE_SUBDIR Build
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG 0e319be336d9192fbd307d2041420f171063b44c
)
FetchContent_MakeAvailable(JoltPhysics)

# Mains
add_executable(repro repro.cpp)
target_link_libraries(repro Jolt Threads::Threads)
