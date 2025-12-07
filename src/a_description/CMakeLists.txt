cmake_minimum_required(VERSION 3.8)
project(a_description)

# コンパイルオプション（おまじない）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ビルドツールの読み込み
find_package(ament_cmake REQUIRED)

# 【最重要】フォルダを install/share/パッケージ名 にコピーする設定
# launch, meshes, urdf フォルダの中身を全部インストールします
install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# パッケージの終了処理
ament_package()