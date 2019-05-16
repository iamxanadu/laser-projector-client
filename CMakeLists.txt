cmake_minimum_required(VERSION 3.14)
project(ILDA_transfer_client)

set(CMAKE_CXX_STANDARD 14)

add_executable(ILDA_transfer_client main.cpp)
target_link_libraries(ILDA_transfer_client lzr cobs)