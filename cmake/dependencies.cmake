cmake_minimum_required(VERSION 3.10)

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include(ExternalProject)

set(source_dir "${CMAKE_BINARY_DIR}/liblidar-src")
set(build_dir "${CMAKE_BINARY_DIR}/liblidar-build")

EXTERNALPROJECT_ADD(
  liblidar
  GIT_REPOSITORY    https://github.com/lukaskaz/lib-lidar.git
  GIT_TAG           main
  PATCH_COMMAND     ""
  PREFIX            liblidar-workspace
  SOURCE_DIR        ${source_dir}
  BINARY_DIR        ${build_dir}
  CONFIGURE_COMMAND mkdir /${build_dir}/build &> /dev/null
  BUILD_COMMAND     cd ${build_dir}/build && cmake -D BUILD_SHARED_LIBS=ON
                    ${source_dir} && make -j 4
  UPDATE_COMMAND    ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)

include_directories(${source_dir}/inc)
link_directories(${build_dir}/build)

set(source_dir "${CMAKE_BINARY_DIR}/libserial-src")
set(build_dir "${CMAKE_BINARY_DIR}/libserial-build")

EXTERNALPROJECT_ADD(
  libserial
  GIT_REPOSITORY    https://github.com/lukaskaz/lib-serial.git
  GIT_TAG           main
  PATCH_COMMAND     ${patching_cmd}
  PREFIX            libserial-workspace
  SOURCE_DIR        ${source_dir}
  BINARY_DIR        ${build_dir}
  CONFIGURE_COMMAND mkdir /${build_dir}/build &> /dev/null
  BUILD_COMMAND     cd ${build_dir}/build && cmake -D BUILD_SHARED_LIBS=ON
                    ${source_dir} && make -j 4
  UPDATE_COMMAND    ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)

include_directories(${source_dir}/inc)
link_directories(${build_dir}/build)

set(source_dir "${CMAKE_BINARY_DIR}/libmenu-src")
set(build_dir "${CMAKE_BINARY_DIR}/libmenu-build")

EXTERNALPROJECT_ADD(
  libmenu
  GIT_REPOSITORY    https://github.com/lukaskaz/lib-menu.git
  GIT_TAG           main
  PATCH_COMMAND     ""
  PREFIX            libmenu-workspace
  SOURCE_DIR        ${source_dir}
  BINARY_DIR        ${build_dir}
  CONFIGURE_COMMAND mkdir /${build_dir}/build &> /dev/null
  BUILD_COMMAND     cd ${build_dir}/build && cmake -D BUILD_SHARED_LIBS=ON
                    ${source_dir} && make -j 4
  UPDATE_COMMAND    ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)

include_directories(${source_dir}/inc)
link_directories(${build_dir}/build)

if(NOT (EXISTS ${CMAKE_BINARY_DIR}/liblogger-src
    OR EXISTS ${CMAKE_BINARY_DIR}/liblogger-build))
    execute_process(
        COMMAND ln -s ${build_dir}/build/liblogger-src ${CMAKE_BINARY_DIR}
        COMMAND ln -s ${build_dir}/build/liblogger-build ${CMAKE_BINARY_DIR}
    )
endif()

set(source_dir "${CMAKE_BINARY_DIR}/liblogger-src")
set(build_dir "${CMAKE_BINARY_DIR}/liblogger-build")
include_directories(${source_dir}/inc)
link_directories(${build_dir}/build)


