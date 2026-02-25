cmake_minimum_required(VERSION 3.22)
project(resource_retriever_service_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(resource_retriever_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME}
  src/resource_retriever_service_plugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  resource_retriever::resource_retriever
  ${resource_retriever_interfaces_TARGETS}
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  resource_retriever
  resource_retriever_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_lint_auto_find_test_dependencies()

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wthread-safety)
  endif()

  ament_add_gmock(test_all test/resource_retriever_service_plugin_test.cpp)
  target_link_libraries(test_all ${PROJECT_NAME})
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
