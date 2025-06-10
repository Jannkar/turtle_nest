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

#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <QString>
#include "turtle_nest/build_type_enum.h"
#include <turtle_nest/packageinfo.h>
#include <turtle_nest/rospkgcreator.h>

bool file_exists(QString path);
bool string_exists_in_file(const QString& filePath, const QString& searchString);
QString get_tmp_workspace_path();
bool dir_is_empty(const QString& dirPath);

// Expects the workspace_path with /src ending
QString run_command(QString command, QStringList outputs_to_wait, QString workspace_path = "");

std::tuple<PackageInfo, RosPkgCreator> create_temp_package(BuildType build_type, QString pkg_name="test_package");
#endif // TEST_UTILS_H
