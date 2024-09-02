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

#ifndef ROSPKGCREATOR_H
#define ROSPKGCREATOR_H

#include "turtle_nest/build_type_enum.h"
#include <QString>
#include <QDir>

class RosPkgCreator
{
private:
  QStringList create_command() const;
  void run_command(QStringList command) const;

public:
  QString workspace_path;
  QString package_name;
  BuildType build_type;
  QString description = "";
  QString maintainer_name = "";

  // ROS automatically populates the email from maintainer name, if email is not provided.
  // If the name contains invalid characters (or spaces) for email, the package building
  // will fail. This is why we need to set this default email instead of leaving it empty.
  QString maintainer_email = "todo@todo.todo";
  QString node_name_cpp = "";
  QString node_name_python = "";
  QString launch_name = "";
  QString params_file_name = "";
  QString license = "TODO: License declaration";
  QString package_path;

  RosPkgCreator(QString workspace_path, QString package_name, BuildType build_type) {
    this->workspace_path = workspace_path;
    this->package_name = package_name;
    this->build_type = build_type;
    this->package_path = QDir(workspace_path).filePath(package_name);
  };

  void create_package() const;
  void build_package() const;
};

#endif // ROSPKGCREATOR_H
