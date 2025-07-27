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

#ifndef MIXED_CPP_PYTHON_PACKAGE_GENERATOR_H
#define MIXED_CPP_PYTHON_PACKAGE_GENERATOR_H

#include "turtle_nest/package_generators/cpp_package_generator.h"


class MixedCppPythonPackageGenerator: public CppPackageGenerator
{
public:
  std::vector < NodeType > get_supported_node_types() const override {
    return {CPP_NODE, CPP_LIFECYCLE_NODE, PYTHON_NODE, PYTHON_LIFECYCLE_NODE};
  }
  void add_node(
    NodeOptions node_options, QString package_path,
    QString package_name) override;
};

void install_python_modules_in_cmakelists(QString cmakelists_path);
void add_python_node_to_cmakelists(QString package_path, QString node_name);


#endif // MIXED_CPP_PYTHON_PACKAGE_GENERATOR_H
