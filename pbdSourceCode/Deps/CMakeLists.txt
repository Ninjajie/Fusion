find_package(Git)
# Enable DownloadProject CMake module
include(DownloadProject)

download_project(
  PROJ partio
  GIT_REPOSITORY https://github.com/wdas/partio.git
  GIT_TAG v1.1.0
  UPDATE_DISCONNECTED 1
  PATCH_COMMAND ${GIT_EXECUTABLE} apply ${CMAKE_CURRENT_SOURCE_DIR}/partio.patch
  )

add_subdirectory(${partio_SOURCE_DIR} ${partio_BINARY_DIR})
set_target_properties(partio PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${partio_SOURCE_DIR}/src/lib"
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${partio_SOURCE_DIR}/src/lib"
    INTERFACE_LINK_LIBRARIES "z"
)