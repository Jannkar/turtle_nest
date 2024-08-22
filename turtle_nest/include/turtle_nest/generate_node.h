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

void generate_python_node(QString workspace_path, QString package_name, QString node_name);
void create_init_file(QString node_dir);
void add_py_node_to_cmake(QString c_make_file_path, QString package_name, QString node_name);
void add_exec_permissions(QString node_path);
void generate_cpp_node(QString package_path, QString node_name);

#endif // GENERATE_NODE_H
