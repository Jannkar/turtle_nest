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

#include "turtle_nest/generate_cmake.h"
#include "turtle_nest/file_utils.h"
#include <QDir>

void modify_cmake_file(
  QString package_path, bool create_launch, bool create_config,
  QString python_node_name)
{
  QString c_make_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";
  if (create_launch) {
    QString launch_append =
      R"(# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

)";
    append_to_file_before(c_make_path, launch_append, append_before_text);
  }

  if (create_config) {
    QString config_append =
      R"(# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

)";
    append_to_file_before(c_make_path, config_append, append_before_text);
  }

  if (!python_node_name.isEmpty()) {
    QString content(
      R"(# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/%1.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME %1
)

)");
    content = content.arg(python_node_name);
    append_to_file_before(c_make_path, content, append_before_text);
  }
}
