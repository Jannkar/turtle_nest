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

#ifndef GENERATE_PARAMS_H
#define GENERATE_PARAMS_H

#include <QString>

void generate_params_file(
  QString package_path, QString params_file_name, QString node_name_cpp,
  QString node_name_python);
QString get_params_content(QString node_name_cpp, QString node_name_python);

#endif // GENERATE_PARAMS_H
