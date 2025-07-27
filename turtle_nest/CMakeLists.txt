cmake_minimum_required(VERSION 3.8)
project(turtle_nest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Svg Concurrent)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(pybind11 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# Set sources
set(SOURCES
  src/addnodedialog.cpp
  src/main.cpp
  src/mainwindow.cpp
  src/file_utils.cpp
  src/generate_cmake.cpp
  src/generate_launch.cpp
  src/generate_msgs_pkg.cpp
  src/generate_params.cpp
  src/generate_setup_py.cpp
  src/modify_existing_pkg.cpp
  src/package_generators/cpp_package_generator.cpp
  src/package_generators/mixed_cpp_python_package_generator.cpp
  src/package_generators/package_generator_factory.cpp
  src/package_generators/python_package_generator.cpp
  src/package_xml_tools.cpp
  src/packageinfoform.cpp
  src/packageswindow.cpp
  src/rospkgcreator.cpp
  src/string_tools.cpp
)

# Set UI files
set(UI_FILES
  src/mainwindow.ui
  src/packageswindow.ui
  src/packageinfoform.ui
  src/addnodedialog.ui
)

# Use Qt5's tools to process the .ui file
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# Add MOC processing for header files
set(MOC_HEADERS
  include/turtle_nest/addnodedialog.h
  include/turtle_nest/mainwindow.h
  include/turtle_nest/packageswindow.h
  include/turtle_nest/packageinfoform.h
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
  $<INSTALL_INTERFACE:include>
  ${Python3_INCLUDE_DIRS}  # Add Python headers
)

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

install(TARGETS turtle_nest
  DESTINATION lib/${PROJECT_NAME})

# Install globally visible executable
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/turtle_nest
  DESTINATION bin
  RENAME turtle-nest)

# Install the Python script
install(FILES src/python_file_utils.py DESTINATION lib/${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::Svg
  Qt5::Concurrent
  ${Python3_LIBRARIES}  # Link to Python libraries
  pybind11::embed
  tinyxml2::tinyxml2
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
