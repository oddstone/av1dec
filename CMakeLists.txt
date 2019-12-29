cmake_minimum_required(VERSION 3.10)

project (av1dec)

include_directories(aom)
add_subdirectory (aom)
add_subdirectory (decoder)
add_subdirectory (tests)