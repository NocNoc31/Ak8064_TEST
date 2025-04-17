cmake_minimum_required(VERSION 3.5)
project(can_interface)

find_package(ament_cmake REQUIRED)

# Tạo thư viện
add_library(can_interface SHARED src/caninterface.cpp)
target_include_directories(can_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Cung cấp thư viện để package khác có thể sử dụng
install(TARGETS can_interface
  EXPORT export_can_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_can_interface HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(can_interface)

ament_package()
