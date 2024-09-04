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

#include "turtle_nest/rospkgcreator.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/generate_cmake.h"
#include "turtle_nest/generate_launch.h"
#include "turtle_nest/generate_node.h"
#include "turtle_nest/generate_params.h"
#include "turtle_nest/generate_setup_py.h"
#include "turtle_nest/string_tools.h"

#include <QDir>
#include <QFile>
#include <QProcess>
#include <QDebug>


void RosPkgCreator::create_package() const
{
  create_directory(workspace_path);
  QStringList command = create_command();
  run_command(command);

  bool create_launch = (launch_name != "");
  bool create_config = (params_file_name != "");

  // Overwrite the simple hello world nodes with more advanced ones
  if (!node_name_python.isEmpty()) {
    generate_python_node(workspace_path, package_name, node_name_python, create_config);
  }
  if (!node_name_cpp.isEmpty()) {
    generate_cpp_node(package_path, node_name_cpp, create_config);
  }

  // Create Python init file if creating CPP and Python package
  if (build_type == CPP_AND_PYTHON) {
    QString node_dir = QDir(package_path).filePath(package_name);
    create_init_file(node_dir);
  }

  // Generate launch file
  if (create_launch) {
    generate_launch_file(
      workspace_path, package_name, launch_name + ".py", params_file_name, node_name_cpp,
      node_name_python);
  }

  // Generate parameters file
  if (create_config) {
    generate_params_file(package_path, params_file_name + ".yaml", node_name_cpp, node_name_python);
  }

  // Modify setup.py or CMakeLists
  if (build_type == PYTHON) {
    modify_setup_py(package_path, create_launch, create_config);
  } else {
    modify_cmake_file(package_path, create_launch, create_config, node_name_python);
  }

  // Add watermark
  QFile file(QDir(package_path).filePath("package.xml"));
  if (file.open(QIODevice::Append | QIODevice::Text)) {
    QTextStream out(&file);
    out << "<!-- This package was generated using the Turtle Nest ROS 2 package creator -->\n";
    file.close();
  } else {
    qDebug() << "Failed to open the package.xml for appending watermark.";
  }

}

QStringList RosPkgCreator::create_command() const
{
  QStringList command_list;
  QString type = "ament_cmake";
  if (build_type == PYTHON) {
    type = "ament_python";
  }

  command_list
    << "pkg" << "create"
    << "--build-type" << type
    << "--license" << license;

  if (!description.trimmed().isEmpty()) {
    command_list << "--description" << escape_xml(description);
  }
  if (!maintainer_name.trimmed().isEmpty()) {
    command_list << "--maintainer-name" << escape_xml(maintainer_name);
  }

  // Set dependencies. This cannot be the last thing before the package name
  // in the command. Otherwise crashes, so seems to be a bug. Having maintainer-email
  // as the last thing because of this.
  if (!node_name_cpp.isEmpty() && !node_name_python.isEmpty()) {
    command_list << "--dependencies" << "rclcpp" << "rclpy" << "std_msgs";
  } else if (!node_name_cpp.isEmpty()) {
    command_list << "--dependencies" << "rclcpp" << "std_msgs";
  } else if (!node_name_python.isEmpty()) {
    command_list << "--dependencies" << "rclpy" << "std_msgs";
  }

  // Set Node name
  if (!node_name_cpp.isEmpty()) {
    command_list << "--node-name" << node_name_cpp;
  }
  if (build_type == PYTHON && !node_name_python.isEmpty()) {
    command_list << "--node-name" << node_name_python;
  }

  command_list << "--maintainer-email" << maintainer_email
               << package_name;
  return command_list;
}

void RosPkgCreator::run_command(QStringList command) const
{
  QProcess builder;
  builder.setProcessChannelMode(QProcess::MergedChannels);
  builder.setWorkingDirectory(workspace_path);
  builder.start("ros2", command);
  qInfo().noquote() << "Running command: ros2" << builder.arguments().join(" ");

  if (!builder.waitForFinished()) {
    throw std::runtime_error("Failed to create a new package. Is ROS 2 installed?");
  }
  if (builder.exitCode() != 0) {
    auto error_message = builder.readAll();
    throw std::runtime_error("Running the command failed: " + error_message);
  }

  qInfo().noquote() << builder.readAll();
}

void RosPkgCreator::build_package() const
{
  QDir dir(workspace_path);
  dir.cdUp();    // Get the path without /src extension to build in that location

  QProcess builder;
  builder.setProcessChannelMode(QProcess::MergedChannels);
  builder.setWorkingDirectory(dir.path());

  QStringList command_list;
  command_list << "build" << "--packages-select" << package_name;

  builder.start("colcon", command_list);
  qInfo().noquote() << "Running command: colcon" << builder.arguments().join(" ");

  if (!builder.waitForFinished()) {
    throw std::runtime_error("Failed to build the package");
  }
  if (builder.exitCode() != 0) {
    auto error_message = builder.readAll();
    throw std::runtime_error("Failed building the package: " + error_message);
  }

  qInfo().noquote() << builder.readAll();
}
