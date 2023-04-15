cmake_minimum_required(VERSION 3.21)
project(jn_ranges
        VERSION 0.0.1
        LANGUAGES CXX)

add_library(jn_ranges INTERFACE include/jn/bit_reshape.hpp include/jn/range_adaptor.hpp)
add_library(JN::Ranges ALIAS jn_ranges)
target_include_directories(jn_ranges INTERFACE include)
target_compile_features(jn_ranges INTERFACE cxx_std_23)

if (PROJECT_IS_TOP_LEVEL)
  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  include(CTest)
  if (BUILD_TESTING)
    add_subdirectory(test)
  endif ()
endif ()
