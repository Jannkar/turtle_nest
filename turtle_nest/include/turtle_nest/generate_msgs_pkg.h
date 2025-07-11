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

#ifndef GENERATE_MSGS_PKG_H
#define GENERATE_MSGS_PKG_H

#include <QString>

void add_msgs_to_cmakelists(QString package_path);
void create_msgs_files(QString package_path);
QString get_msgs_cmake_addition();
QString get_example_msg_contents();

#endif // GENERATE_MSGS_PKG_H
