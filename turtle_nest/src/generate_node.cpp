/* ------------------------------------------------------------------
 * Copyright 2024 Janne Karttunen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------
*/

// Include Python headers. Avoiding build issue as shown here:
// https://stackoverflow.com/questions/23068700/embedding-python3-in-qt-5
#pragma push_macro("slots")
#undef slots
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#pragma pop_macro("slots")

#include "turtle_nest/generate_node.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/string_tools.h"

#include <QDir>
#include <QDebug>
#include <QRegularExpression>

#include <turtle_nest/package_xml_tools.h>

namespace py = pybind11;


void generate_python_node(
  QString package_path, QString package_name, QString node_name,
  bool create_config, bool overwrite_existing)
{
  create_init_file(package_path, package_name);
  QString node_dir = QDir(package_path).filePath(package_name);
  QString node_path = QDir(node_dir).filePath(node_name + ".py");

  // Block to be added if parameter file was created
  QString param_declare_block =
    !create_config ? "" :
    R"(
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}"))";

  // Main content
  QString content = QString(
    R"(#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class %2(Node):

    def __init__(self):
        super().__init__("%1")%3
        self.get_logger().info("Hello world from the Python node %1")


def main(args=None):
    rclpy.init(args=args)

    %1 = %2()

    try:
        rclpy.spin(%1)
    except KeyboardInterrupt:
        pass

    %1.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
)")
    .arg(node_name, to_camel_case(node_name), param_declare_block);

  create_directory(node_dir);
  write_file(node_path, content, overwrite_existing);
  qDebug() << "Generated new Python node " + node_name;
  add_exec_permissions(node_path);
  add_rclpy_dependency_to_package_xml(package_path);
}

void create_init_file(QString package_path, QString package_name)
{
  QString node_dir = QDir(package_path).filePath(package_name);
  QString init_path = QDir(node_dir).filePath("__init__.py");
  if (QFile::exists(init_path)) {
    return;
  }

  create_directory(node_dir);
  write_file(init_path, "");
}

void add_exec_permissions(QString node_path)
{
  QFile file(node_path);
  QFileDevice::Permissions current_permissions = file.permissions();

  QFileDevice::Permissions new_permissions = current_permissions |
    QFileDevice::ExeOwner |
    QFileDevice::ExeGroup |
    QFileDevice::ExeOther;

  if (!file.setPermissions(new_permissions)) {
    QString error_msg = QString("Failed to set execution permissions for Python file '%1'").arg(
      node_path);
    qCritical() << error_msg;
    throw std::runtime_error(error_msg.toStdString());
  }
}


void generate_cpp_node(
  QString package_path, QString node_name, bool create_config,
  bool overwrite_existing)
{
  QString params_block =
    !create_config ? "" :
    R"(
    this->declare_parameter<std::string>("example_param", "default_value");
    std::string example_param = this->get_parameter("example_param").as_string();
    RCLCPP_INFO(
      this->get_logger(), "Declared parameter 'example_param'. Value: %s", example_param.c_str());
)";
  // Uncrustify considers this as a single line. Skip.
  /* *INDENT-OFF* */

    QString content = QString(R"(#include "rclcpp/rclcpp.hpp"

class %2 : public rclcpp::Node
{
public:
  %2()
  : Node("%1")
  {%3
    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "%1");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<%2>());
  rclcpp::shutdown();
  return 0;
}
)").arg(node_name, to_camel_case(node_name), params_block);
  /* *INDENT-ON* */
  write_file(QDir(package_path).filePath("src/" + node_name + ".cpp"), content, overwrite_existing);
  add_rclcpp_dependency_to_package_xml(package_path);
}


void add_node_to_cmakelists(PackageInfo pkg_info, QString node_name)
{
  QString cmakelists_path = QDir(pkg_info.package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";

  add_dependency_to_cmakelists("rclcpp", cmakelists_path);

  QString content(
    R"(# Add node %1
add_executable(%1 src/%1.cpp)

target_include_directories(%1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  %1
  "rclcpp"
)

install(TARGETS %1
  DESTINATION lib/${PROJECT_NAME})

)");
  content = content.arg(node_name);
  append_to_file_before(cmakelists_path, content, append_before_text);
}

void add_dependency_to_cmakelists(QString dependency, QString cmakelists_path)
{
  QString target_string = QString("find_package(%1 REQUIRED)").arg(dependency);
  QString cmakelists_contents = read_file(cmakelists_path);

  // Return if dependency already exists
  if (cmakelists_contents.contains(target_string)) {
    return;
  }

  QString append_before_text = "ament_package()";
  append_to_file_before(cmakelists_path, target_string + "\n\n", append_before_text);
}

void install_python_modules_in_cmakelists(QString package_path)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");
  QString cmakelists_contents = read_file(cmakelists_path);
  QString target_string = QString("ament_python_install_package(${PROJECT_NAME})");

  // Return if ament_python_install_package already exists
  if (cmakelists_contents.contains(target_string)) {
    return;
  }

  QString content(
    R"(# Install Python modules
ament_python_install_package(${PROJECT_NAME})

)");

  QString append_before_text = "ament_package()";
  append_to_file_before(cmakelists_path, content, append_before_text);
}

void add_python_node_to_cmakelists(QString package_path, QString node_name)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";

  add_dependency_to_cmakelists("rclpy", cmakelists_path);
  install_python_modules_in_cmakelists(package_path);

  QString content(
    R"(# Add Python node %1
install(PROGRAMS
  ${PROJECT_NAME}/%1.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME %1
)

)");
  content = content.arg(node_name);
  append_to_file_before(cmakelists_path, content, append_before_text);
}

void add_rclpy_dependency_to_package_xml(QString package_path)
{
  PackageXMLEditor xml_editor = PackageXMLEditor(package_path);

  // If rclpy is already exec_depend or depend, don't add.
  if (xml_editor.has_dependency("rclpy", DependencyType::DEPEND)) {
    return;
  }
  if (xml_editor.has_dependency("rclpy", DependencyType::EXEC_DEPEND)) {
    return;
  }

  xml_editor.add_dependency("rclpy", DependencyType::EXEC_DEPEND);
}

void add_rclcpp_dependency_to_package_xml(QString package_path)
{
  PackageXMLEditor xml_editor = PackageXMLEditor(package_path);
  bool has_depend = xml_editor.has_dependency("rclcpp", DependencyType::DEPEND);
  if (has_depend) {
    qDebug() << "Already has dependency rclcpp!";
    return;
  }

  bool has_exec_depend = xml_editor.has_dependency("rclcpp", DependencyType::EXEC_DEPEND);
  bool has_build_depend = xml_editor.has_dependency("rclcpp", DependencyType::BUILD_DEPEND);

  if (has_exec_depend && has_build_depend) {
    return;
  } else if (has_exec_depend && !has_build_depend) {
    xml_editor.add_dependency("rclcpp", DependencyType::BUILD_DEPEND);
  } else if (!has_exec_depend && has_build_depend) {
    xml_editor.add_dependency("rclcpp", DependencyType::EXEC_DEPEND);
  } else {
    xml_editor.add_dependency("rclcpp", DependencyType::DEPEND);
  }
}


//
/**
 * @brief Modifies setup.py file by adding a Node into it as an entrypoint
 */
void add_node_to_setup_py(PackageInfo pkg_info, QString node_name)
{
  QString setup_path = QDir(pkg_info.package_path).filePath("setup.py");
  QString new_code = generate_new_setup_py(pkg_info, node_name);
  write_file(setup_path, new_code, true);
}

/**
 * @brief Calls a Python script to generate the new setup_py. Python script
 * parses the abstract syntrax tree of the file, to append to the console_scripts
 * correctly, no matter what the code structure is.
 */
QString generate_new_setup_py(PackageInfo pkg_info, QString node_name)
{
  QString module_name = "python_file_utils";
  QString function_name = "generate_new_setup_py";
  static py::scoped_interpreter guard{};

  QDir dir(get_executable_dir());

  // A dirty way of getting the Python executable path. Our app
  // location will be "bin" when running with turtle-nest, "lib"
  // with ros2 run turtle_nest turtle_nest. With Qt editor build,
  // we don't have "bin" or "lib" folders.
  if (dir.dirName() == "bin" || dir.dirName() == "lib") {
    dir.cdUp();      // Move up from install/bin to install/
  }
  QString script_path = dir.filePath("lib/turtle_nest/");

  // Another dirty way of getting the Python file in tests.
  // Should be reworked to have the tests under the ROS 2 package,
  // to make this easier.
  QDir checkDir(script_path);
  if (!checkDir.exists()) {
    script_path = dir.filePath("lib/turtle_nest_tests/");
  }

  py::module sys = py::module::import("sys");
  sys.attr("path").cast<py::list>().append(script_path.toStdString());

  py::module module = py::module::import(module_name.toStdString().c_str());
  py::object func = module.attr(function_name.toStdString().c_str());

  try {
    py::object result = func(
      py::str(pkg_info.package_path.toStdString()),
      py::str(pkg_info.package_name.toStdString()),
      py::str(node_name.toStdString())
    );

    return QString().fromStdString(result.cast<std::string>());
  } catch (const py::error_already_set & e) {
    throw std::runtime_error(
            QString(
              "Failed to parse the package setup.py! Is it valid and does it have "
              "entry_points and console_scripts?").toStdString()
    );
  }
}
