cmake_minimum_required(VERSION 3.8)
project(turtle_nest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)


# Set sources
set(SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/file_utils.cpp
  src/generate_launch.cpp
  src/generate_node.cpp
  src/rospkgcreator.cpp
  src/string_tools.cpp
)

# Set UI files
set(UI_FILES
  src/mainwindow.ui
)

# Use Qt5's tools to process the .ui file
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# Add MOC processing for header files
set(MOC_HEADERS
  include/turtle_nest/mainwindow.h
)

qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

# Set resource files
set(RESOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
)

# Compile resource files into object files
qt5_add_resources(QRC_SOURCES ${RESOURCE_FILES})


# Add the executable
add_executable(turtle_nest ${SOURCES} ${UI_HEADERS} ${MOC_SOURCES} ${QRC_SOURCES})

target_include_directories(turtle_nest PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>  # Include directory for generated ui_mainwindow.h
  $<INSTALL_INTERFACE:include>)

# List of distribution files
set(DISTFILES
    ${CMAKE_CURRENT_SOURCE_DIR}/custom_theme.qss
)

# Add a custom command to copy distribution files to the build directory
foreach(file ${DISTFILES})
    add_custom_command(TARGET turtle_nest
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${file} $<TARGET_FILE_DIR:turtle_nest>
    )
endforeach()


target_compile_features(turtle_nest PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  turtle_nest
)

install(TARGETS turtle_nest
  DESTINATION lib/${PROJECT_NAME})

# Install globally visible executable
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/turtle_nest
  DESTINATION bin
  RENAME turtle-nest)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
