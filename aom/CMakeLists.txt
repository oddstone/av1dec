file(GLOB all_files
    "*.h"
    "*.c")

add_library(libaom OBJECT ${all_files})