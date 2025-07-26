/* ------------------------------------------------------------------
 * Copyright 2025 Janne Karttunen
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

#include <QDir>
#include "turtle_nest/package_generators/python_package_generator.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/string_tools.h"
#include "turtle_nest/package_xml_tools.h"

namespace py = pybind11;

void PythonPackageGenerator::add_node(
  NodeOptions node_options, QString package_path,
  QString package_name)
{
  if (node_options.node_type == PYTHON_NODE) {
    generate_python_node(package_path, package_name, node_options.node_name, node_options.add_params);
    add_node_to_setup_py(package_path, package_name, node_options.node_name);
  } else {
    BasePackageGenerator::add_node(node_options, package_path, package_name);
  }
}

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

//
/**
 * @brief Modifies setup.py file by adding a Node into it as an entrypoint
 */
void add_node_to_setup_py(QString package_path, QString package_name, QString node_name)
{
  QString setup_path = QDir(package_path).filePath("setup.py");
  QString new_code = generate_new_setup_py(package_path, package_name, node_name);
  write_file(setup_path, new_code, true);
}

/**
 * @brief Calls a Python script to generate the new setup_py. Python script
 * parses the abstract syntrax tree of the file, to append to the console_scripts
 * correctly, no matter what the code structure is.
 */
QString generate_new_setup_py(QString package_path, QString package_name, QString node_name)
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
      py::str(package_path.toStdString()),
      py::str(package_name.toStdString()),
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
