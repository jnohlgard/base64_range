add_library(Boost::ut IMPORTED INTERFACE)
target_include_directories(Boost::ut INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/ut/include")

add_executable(bit_reshape_test bit_reshape_test.cpp test_main.cpp)
target_link_libraries(bit_reshape_test PRIVATE JN::Ranges Boost::ut)
add_test(NAME bit_reshape_test COMMAND bit_reshape_test)
