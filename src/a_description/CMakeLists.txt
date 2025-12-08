cmake_minimum_required(VERSION 3.14.4)
project(a_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# meshes, urdf, launch を install/share/a_description にコピー
install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# package.xml もコピー
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()