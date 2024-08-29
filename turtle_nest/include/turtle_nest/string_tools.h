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

#ifndef STRING_TOOLS_H
#define STRING_TOOLS_H

#include <QString>
#include <QLineEdit>

QString escape_xml(QString input);
bool is_valid_alphanumeric(QString string);
QString autocorrect_line_edit(QString text, QLineEdit * line_edit);
QString autocorrect_string(QString string);
bool is_valid_email(QString email);
QString to_camel_case(const QString & snake_case);

#endif // STRING_TOOLS_H
