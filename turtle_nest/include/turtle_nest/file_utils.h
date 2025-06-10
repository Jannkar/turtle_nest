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

#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <QString>

void create_directory(QString path);
QString read_file(QString path);
void write_file(QString path, QString content, bool overwrite = false);
void append_to_file_before(QString file_path, QString lines_to_append, QString append_before_text);
void append_to_file(QString file_path, QString lines_to_append, QString append_after_text);
QString read_xml_tag(const QString &filePath, const QString &tagName, const QString &attributeName = QString());
QString get_executable_dir();
QString get_workspace_path_with_src(QString path);
QString get_workspace_path_without_src(QString path);

#endif // FILE_UTILS_H
