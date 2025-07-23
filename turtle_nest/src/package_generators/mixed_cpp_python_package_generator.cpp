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

#include "turtle_nest/package_generators/mixed_cpp_python_package_generator.h"
#include "turtle_nest/package_generators/cpp_package_generator.h"
#include "turtle_nest/package_generators/python_package_generator.h"
#include "turtle_nest/file_utils.h"


void MixedCppPythonPackageGenerator::add_node(
  NodeOptions node_options,
  QString package_path, QString package_name)
{
  if (node_options.node_type == PYTHON_NODE) {
    generate_python_node(package_path, package_name, node_options.node_name, false);
    add_python_node_to_cmakelists(package_path, node_options.node_name);
  }
  else{
    CppPackageGenerator::add_node(node_options, package_path, package_name);
  }
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
