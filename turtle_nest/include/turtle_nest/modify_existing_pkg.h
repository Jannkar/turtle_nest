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

#ifndef MODIFY_EXISTING_PKG_H
#define MODIFY_EXISTING_PKG_H

#include "turtle_nest/node_type_enum.h"
#include "turtle_nest/packageinfo.h"
#include <QString>
#include <QDir>

std::map < QString, PackageInfo > list_packages(QString workspace_path);
QStringList list_executables(const QString workspace_path, const QString package_name);
QStringList list_files(const QString path);
bool is_src_package(QDir dir);
bool is_python_package(QString package_path);
bool is_mixed_package(QString package_path);
bool is_cpp_package(QString package_path);
bool is_msgs_package(QString package_name);
BuildType get_package_build_type(QString package_path, QString package_name);
QString clean_description(const QString & input);
void add_node(NodeOptions node_options, PackageInfo pkg_info);

#endif // MODIFY_EXISTING_PKG_H
