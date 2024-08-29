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

#include "turtle_nest/generate_launch.h"

#include "turtle_nest/file_utils.h"
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include <QDebug>
#include <sstream>


void generate_launch_file(
  QString workspace_path, QString package_name, QString launch_file_name,
  QString node_name_cpp, QString node_name_python, BuildType build_type)
{
  QString launch_text = generate_launch_text(package_name, node_name_cpp, node_name_python);
  QString launch_file_dir = QDir(workspace_path).filePath(package_name + "/launch/");
  QString launch_file_path = QDir(launch_file_dir).filePath(launch_file_name);

  create_directory(launch_file_dir);
  write_file(launch_file_path, launch_text);
  qInfo() << "Created launch file " << launch_file_path;

  if (build_type == PYTHON) {
    QString setup_py_path = QDir(workspace_path).filePath(package_name + "/setup.py");
    append_launch_to_setup_py(setup_py_path, package_name);
  } else {
    QString c_make_file_path = QDir(workspace_path).filePath(package_name + "/CMakeLists.txt");
    append_launch_to_cmake(c_make_file_path);
  }
}

QString generate_launch_text(QString package_name, QString node_name_cpp, QString node_name_python)
{
  std::ostringstream oss;
  oss <<
    R"(from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([)";

  if (!node_name_cpp.isEmpty()) {
    oss << R"(
        Node(
            package=')" << package_name.toStdString() << R"(',
            executable=')" << node_name_cpp.toStdString() << R"(',
            name=')" << node_name_cpp.toStdString() <<
      R"(',
            output='screen',
        ),)";
  }

  if (!node_name_python.isEmpty()) {
    oss << R"(
        Node(
            package=')" << package_name.toStdString() << R"(',
            executable=')" << node_name_python.toStdString() << R"(',
            name=')" << node_name_python.toStdString() <<
      R"(',
            output='screen',
        ),)";
  }
  oss << R"(
    ])
)";

  return QString::fromStdString(oss.str());
}

void append_launch_to_cmake(QString c_make_path)
{
  QString lines_to_append =
    R"(# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

)";
  append_to_file_before(c_make_path, lines_to_append, "ament_package()");
}

void append_launch_to_setup_py(QString setup_py_path, QString package_name)
{
  QString lines_to_append(
    "(os.path.join('share', '%1', 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),\n        ");
  lines_to_append = lines_to_append.arg(package_name);
  append_to_file_before(
    setup_py_path, lines_to_append,
    "('share/ament_index/resource_index/packages'");

  QString imports("import os\n"
    "from glob import glob\n");
  append_to_file_before(setup_py_path, imports, "from setuptools import");
}
