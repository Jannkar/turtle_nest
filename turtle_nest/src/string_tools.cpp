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

#include "turtle_nest/string_tools.h"

#include <QRegularExpression>

QString escape_xml(const QString input)
{
  QString output = input;
  output.replace("&", "&amp;");
  output.replace("<", "&lt;");
  output.replace(">", "&gt;");
  output.replace("\"", "");    // Quotation marks are just removed
  output.replace("'", "&apos;");
  return output;
}

bool is_valid_email(QString email)
{
  if (email.length() == 0) {
    return true;      // Email is optional so empty is considered valid
  }
  // Email validation from
  // https://github.com/ros-infrastructure/catkin_pkg/blob/master/src/catkin_pkg/package.py
  static const QRegularExpression regex(
    R"(^[-a-zA-Z0-9_%+]+(\.[-a-zA-Z0-9_%+]+)*@[-a-zA-Z0-9%]+(\.[-a-zA-Z0-9%]+)*\.[a-zA-Z]{2,}$)");
  QRegularExpressionMatch match = regex.match(email);
  return match.hasMatch();
}

// Check if the QString only contains lowercase letters and underscores
bool is_valid_alphanumeric(QString string)
{
  // This regular expression checks if the entire string consists of one or
  // more lowercase letters, digits, or underscores, and nothing else.
  // And starts with a letter.
  static const QRegularExpression valid_input("^[a-z][a-z0-9_]*$");
  return valid_input.match(string).hasMatch();
}

QString autocorrect_line_edit(QString text, QLineEdit * line_edit)
{
  int cursor_pos = line_edit->cursorPosition();
  QString autocorrected_text = autocorrect_string(text);
  line_edit->setText(autocorrected_text);

  // Set cursor back to right position
  line_edit->setCursorPosition(cursor_pos - (text.length() - autocorrected_text.length()));
  return autocorrected_text;
}

QString autocorrect_string(QString string)
{
  // Remove leading non-letter characters
  static const QRegularExpression leading_non_letters("^[^a-z]+");
  string = string.toLower().replace(leading_non_letters, "");

  // Convert string to lower, replaces spaces with underscore (_) and removes any invalid characters
  static const QRegularExpression invalid_characters("[^a-z0-9_]");
  return string.toLower().replace(" ", "_").remove(invalid_characters);
}


QString to_camel_case(const QString & snake_case)
{
  QStringList words = snake_case.split('_');
  QString camel_case;

  for (const QString & word : words) {
    camel_case += word.left(1).toUpper() + word.mid(1).toLower();
  }

  return camel_case;
}
