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

#ifndef PACKAGEINFO_H
#define PACKAGEINFO_H

#include "turtle_nest/build_type_enum.h"
#include <QDir>
#include <QString>


class PackageInfo
{
public:
  PackageInfo() {
  };
  QString package_name;
  QString package_path;  // Package path, for example ~/ros2_ws/src/repo/my_package
  QString package_destination;  // One layer above the package path, not necessarily a workspace. For example ~/ros2_ws/src/repo
  QString maintainer = "";
  QString maintainer_email = "";
  QString description = "";
  QString version = "";
  QString license = "";
  BuildType package_type;

  // Constructor: user provides package_name, destination, and type
  PackageInfo(const QString& name,
              const QString& destination,
              BuildType type)
      : package_name(name),
      package_destination(destination),
      package_type(type)
  {
    // Compute package_path automatically. This can be done automatically when package_name is the same as the
    // package path last segment, but might not be true in all the cases. 
    // In this case, we can use the default constructor and set package_path manually later.
    QDir dest_dir(destination);
    package_path = dest_dir.filePath(package_name);
  }
};

#endif // PACKAGEINFO_H
