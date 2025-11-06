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
#ifndef PYTHON_NODE_GENERATOR_H
#define PYTHON_NODE_GENERATOR_H

#include "turtle_nest/node_generators/base_node_generator.h"


class PythonNodeGenerator: public BaseNodeGenerator
{
public:
  std::vector < NodeType > get_supported_node_types() const override {
    return {PYTHON_NODE, PYTHON_LIFECYCLE_NODE};
  }
  void add_node(
    NodeOptions node_options, QString package_path,
    QString package_name) override;
};

void generate_python_node(
  QString package_path, QString package_name, QString node_name,
  bool create_config, bool overwrite_existing = false, bool lifecycle_node = false);
QString get_python_node_content(QString node_name, QString param_declare_block);
QString get_python_lifecycle_node_content(QString node_name, QString param_declare_block);
QString get_param_declare_block();
void create_init_file(QString package_path, QString package_name);
void add_exec_permissions(QString node_path);
void add_python_pkg_dependency_to_package_xml(QString package_path, QString dependency);
void add_node_to_setup_py(QString package_path, QString package_name, QString node_name);
QString generate_new_setup_py(QString package_path, QString package_name, QString node_name);

#endif // PYTHON_NODE_GENERATOR_H
