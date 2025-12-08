cmake_minimum_required(VERSION 3.14.4)

# 【変更点1】プロジェクト名（パッケージ名）
# ここを新しい名前に変えます（例: my_robot_description）
project(a_description) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 【変更点2】インストールするフォルダのリスト
# もし "config" や "worlds" などのフォルダが増えたら、ここに書き足します
install(
  DIRECTORY launch meshes urdf  # ← ここにフォルダ名を追記する
  DESTINATION share/${PROJECT_NAME}
)

ament_package()