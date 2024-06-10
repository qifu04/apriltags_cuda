include(CheckClangFormat)
check_clang_format(CLANG_FORMAT_CMD)

add_custom_target(clangformat
  COMMAND ${CLANG_FORMAT_CMD} -style=file .clang-format
  DEPENDS ${CMAKE_SOURCE_DIR}/CMakeLists.txt
)
