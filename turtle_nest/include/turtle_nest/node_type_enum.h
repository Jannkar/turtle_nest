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

#ifndef NODE_TYPE_ENUM_H
#define NODE_TYPE_ENUM_H

#include <QString>

enum NodeType
{
  CPP_NODE,
  CPP_LIFECYCLE_NODE,
  PYTHON_NODE,
};

struct NodeOptions
{
  QString node_name;
  NodeType node_type;
  bool add_params;
};

// Convert NodeType enum to user-readable QString
inline QString node_type_to_string(NodeType node_type) {
  switch (node_type) {
    case NodeType::CPP_NODE: return QStringLiteral("C++ Node");
    case NodeType::CPP_LIFECYCLE_NODE: return QStringLiteral("C++ Lifecycle Node");
    case NodeType::PYTHON_NODE: return QStringLiteral("Python Node");
    default:
      throw std::runtime_error(
        QString("node_type_to_string: Not implemented for enum value %1")
              .arg(static_cast<int>(node_type))
              .toStdString());
  }
}

// Convert QString back to NodeType enum (case-sensitive)
inline std::optional<NodeType> node_type_from_string(const QString& s) {
  if (s == QStringLiteral("C++ Node")) return NodeType::CPP_NODE;
  if (s == QStringLiteral("C++ Lifecycle Node")) return NodeType::CPP_LIFECYCLE_NODE;
  if (s == QStringLiteral("Python Node")) return NodeType::PYTHON_NODE;
  throw std::runtime_error(
    QString("node_type_from_string: Not implemented for string value value %1")
          .arg(s)
          .toStdString());;
}

#endif // NODE_TYPE_ENUM_H
