file(GLOB all_files
    "*.h"
    "*.cpp"
	"*.c")

include_directories(${PROJECT_SOURCE_DIR}/interface/
    ${PROJECT_SOURCE_DIR}/common/
    ${PROJECT_SOURCE_DIR}/decoder/)

add_executable(av1dec ${all_files})

target_link_libraries(av1dec libav1dec)