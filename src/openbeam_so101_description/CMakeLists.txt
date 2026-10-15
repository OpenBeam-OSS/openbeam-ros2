cmake_minimum_required(VERSION 3.8)
project(openbeam_so101_description NONE)
find_package(ament_cmake REQUIRED)

# meshes/ is filled by fetch_meshes.sh (checked against meshes/SHA256SUMS);
# refuse to install a description whose meshes are missing.
file(STRINGS meshes/SHA256SUMS _mesh_lines)
foreach(_line IN LISTS _mesh_lines)
  string(REGEX REPLACE "^[0-9a-f]+  " "" _mesh "${_line}")
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes/${_mesh}")
    message(FATAL_ERROR "meshes/${_mesh} is missing: run ./fetch_meshes.sh first")
  endif()
endforeach()

install(DIRECTORY urdf meshes rviz launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE-SO-ARM100 NOTICE DESTINATION share/${PROJECT_NAME})
ament_package()
