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


void generate_launch_file(
  QString workspace_path, QString package_name, QString launch_file_name, QString params_file_name,
  QString node_name_cpp, QString node_name_python)
{
  QString launch_text = generate_launch_text(
    package_name, node_name_cpp, node_name_python,
    params_file_name);
  QString launch_file_dir = QDir(workspace_path).filePath(package_name + "/launch/");
  QString launch_file_path = QDir(launch_file_dir).filePath(launch_file_name);

  create_directory(launch_file_dir);
  write_file(launch_file_path, launch_text);
  qInfo() << "Created launch file " << launch_file_path;
}

QString generate_launch_text(
  QString package_name, QString node_name_cpp, QString node_name_python,
  QString params_file_name)
{
  QString config_import_block =
    params_file_name.isEmpty() ? "" :
    R"(import os
from ament_index_python.packages import get_package_share_directory
)";

  QString config_block = params_file_name.isEmpty() ? "" : QString(
    R"(
    config = os.path.join(
        get_package_share_directory('%1'),
        'config',
        '%2.yaml',
    )
)")
    .arg(package_name, params_file_name);

  QString node_cpp_block = node_name_cpp.isEmpty() ? "" : QString(
    R"(
        Node(
            package='%1',
            executable='%2',
            name='%2',
            output='screen',
            parameters=[%3],
        ),)")
    .arg(package_name, node_name_cpp, config_block.isEmpty() ? "" : "config");

  QString node_python_block = node_name_python.isEmpty() ? "" : QString(
    R"(
        Node(
            package='%1',
            executable='%2',
            name='%2',
            output='screen',
            parameters=[%3],
        ),)")
    .arg(package_name, node_name_python, config_block.isEmpty() ? "" : "config");

  QString launch_content = QString(
    R"(%1
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():%2
    return LaunchDescription([%3%4
    ])
)")
    .arg(config_import_block, config_block, node_cpp_block, node_python_block);
  return launch_content;
}
