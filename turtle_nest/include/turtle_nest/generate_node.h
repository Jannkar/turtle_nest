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

#ifndef GENERATE_NODE_H
#define GENERATE_NODE_H

#include <QString>
#include "turtle_nest/packageinfo.h"


void generate_python_node(
  QString package_path, QString package_name, QString node_name,
  bool create_config, bool overwrite_existing=false);
void create_init_file(QString package_path, QString package_name);
void add_exec_permissions(QString node_path);
void generate_cpp_node(QString package_path, QString node_name,
                       bool create_config, bool overwrite_existing=false);
void add_node_to_cmakelists(PackageInfo pkg_info, QString node_name);
void add_dependency_to_cmakelists(QString dependency, QString cmakelists_path);
void add_rclpy_dependency_to_package_xml(QString package_path);
void add_rclcpp_dependency_to_package_xml(QString package_path);
void install_python_modules_in_cmakelists(QString cmakelists_path);
void add_python_node_to_cmakelists(QString package_path, QString node_name);
void add_node_to_setup_py(PackageInfo pkg_info, QString node_name);
QString generate_new_setup_py(PackageInfo pkg_info, QString node_name);

#endif // GENERATE_NODE_H
