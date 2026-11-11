cmake_minimum_required(VERSION 3.16)
project(aquarm_sdk VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen is part of the public API (header-only, needed at build time only)
find_package(Eigen3 REQUIRED)

add_library(aquarm_sdk SHARED IMPORTED GLOBAL)
set_target_properties(aquarm_sdk PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libkalman_ik.so.${PROJECT_VERSION}
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(aquarm_sdk INTERFACE Eigen3::Eigen)
target_compile_definitions(aquarm_sdk INTERFACE KALMAN_IK_VERSION="${PROJECT_VERSION}")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/example/cpp)
  find_package(Threads REQUIRED)
  add_library(kcan_sdk_user SHARED IMPORTED)
  set_target_properties(kcan_sdk_user PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libkalman_kcan_user.so.7.0.3"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(kcan_sdk_user INTERFACE Threads::Threads)

  add_executable(ik_kcan example/cpp/ik_kcan.cpp)
  target_link_libraries(ik_kcan PRIVATE aquarm_sdk kcan_sdk_user)
  target_compile_definitions(ik_kcan PRIVATE
    AQUARM_URDF="${CMAKE_CURRENT_SOURCE_DIR}/resources/urdf/aquarm_origin_with_gripper.urdf")

  add_executable(joint_kcan example/cpp/joint_kcan.cpp)
  target_link_libraries(joint_kcan PRIVATE kcan_sdk_user)
  set_target_properties(ik_kcan joint_kcan PROPERTIES BUILD_RPATH "\$ORIGIN/../lib")
endif()

# Tests
enable_testing()
foreach(test_name test_ik test_trajectory)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.cpp)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} aquarm_sdk)

    set_target_properties(${test_name} PROPERTIES
      BUILD_RPATH "\$ORIGIN/../lib"
    )
    # test_ik reads resources/urdf/ relative to the SDK root
    add_test(NAME ${test_name} COMMAND ${test_name}
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
endforeach()
