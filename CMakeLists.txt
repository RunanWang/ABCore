cmake_minimum_required(VERSION 3.16)
project(ABCore LANGUAGES CXX)
# 导出 compile_commands.json，供 clangd 解析包含路径并支持跳转。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
find_package(highs CONFIG QUIET)
if(NOT TARGET highs::highs)
    include(FetchContent)
    set(BUILD_CXX_EXE OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(highs
        GIT_REPOSITORY https://github.com/ERGO-Code/HiGHS.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(highs)
    set(CMAKE_CXX_STANDARD 17)
endif()
add_library(abcore
    DataStructure/Graph.cpp DataStructure/Subgraph.cpp
    ConstraintGeneration/ComponentPeeling.cpp
    ConstraintGeneration/ConstraintEncoder.cpp
    ConstraintGeneration/ExactConfig.cpp
    ConstraintGeneration/Solver.cpp
    ConstraintGeneration/GrowthWitnessGenerator.cpp
    ConstraintGeneration/GrowthWitnessSelector.cpp
    ConstraintGeneration/HittingSetModel.cpp
    ConstraintGeneration/Preprocessor.cpp
    ConstraintGeneration/SlackWitnessGenerator.cpp
    ConstraintGeneration/SlackWitnessSelector.cpp
    ConstraintGeneration/Types.cpp
    ConstraintGeneration/WitnessPipeline.cpp
    ExchangeSearch/Solver.cpp ExchangeSearch/ExchangeSearch.cpp
    ExchangeSearch/Greedy.cpp ExchangeSearch/Peeling.cpp
    ExchangeSearch/Preprocessor.cpp ExchangeSearch/RepairEnumerator.cpp)
target_include_directories(abcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(abcore PRIVATE highs::highs)
target_compile_features(abcore PUBLIC cxx_std_17)
add_executable(abcore_case main.cpp)
target_link_libraries(abcore_case PRIVATE abcore)
enable_testing()
add_test(NAME citeseer_case COMMAND abcore_case ${CMAKE_CURRENT_SOURCE_DIR}/Dataset/citeseer)
