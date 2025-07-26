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

#ifndef BASE_PACKAGE_GENERATOR_H
#define BASE_PACKAGE_GENERATOR_H

#include <QDir>
#include <QString>
#include <QDebug>
#include "turtle_nest/node_type_enum.h"

class BasePackageGenerator
{
public:
  virtual ~BasePackageGenerator() = default;
  virtual std::vector < NodeType > get_supported_node_types() const {
    return {};
  }
  virtual void add_node(
    NodeOptions node_options, QString /*package_path*/, QString /*package_name*/)
  {
    throw std::runtime_error(
      QString("Unsupported node type: %1")
      .arg(node_options.node_type)
      .toStdString()
    );
  }
};

#endif // BASE_PACKAGE_GENERATOR_H
