file(GLOB all_files
    "*.h"
    "*.c")

add_library(aom OBJECT ${all_files})