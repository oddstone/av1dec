file(GLOB all_files
    "*.h"
    "*.cpp")

include_directories(${PROJECT_SOURCE_DIR}/aom/
    ${PROJECT_SOURCE_DIR}/common/
    ${PROJECT_SOURCE_DIR}/interface/)


add_library(libav1dec
    ${all_files}
    $<TARGET_OBJECTS:libaom>)