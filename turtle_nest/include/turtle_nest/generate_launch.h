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

#ifndef GENERATE_LAUNCH_H
#define GENERATE_LAUNCH_H

#include <QString>
#include <turtle_nest/node_type_enum.h>


void generate_launch_file(
  QString generate_launch_file, QString package_name, QString launch_file_name,
  QString params_file_name,
  bool composable_launch, QString node_name = "");
QString generate_launch_text(
  QString package_name, QString node_name_cpp, bool composable_launch, QString params_file_name);
void save_launch_file(QString file_name, QString launch_text);

#endif // GENERATE_LAUNCH_H
