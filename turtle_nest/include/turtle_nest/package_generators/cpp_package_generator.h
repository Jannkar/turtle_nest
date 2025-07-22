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

#ifndef CPP_PACKAGE_GENERATOR_H
#define CPP_PACKAGE_GENERATOR_H

#include "turtle_nest/package_generators/base_package_generator.h"


class CppPackageGenerator : public BasePackageGenerator
{
public:
  void add_node(QString node_name, NodeType node_type, QString package_path, QString package_name) override;
};

void generate_cpp_node(
    QString package_path, QString node_name,
    bool create_config, bool overwrite_existing = false);
void add_node_to_cmakelists(QString package_path, QString node_name);
void add_dependency_to_cmakelists(QString dependency, QString cmakelists_path);
void add_rclcpp_dependency_to_package_xml(QString package_path);

#endif // CPP_PACKAGE_GENERATOR_H
