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

#include "turtle_nest/file_utils.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>


void create_directory(QString path)
{
  QDir dir(path);
  if (dir.exists()) {
    qInfo() << "Skipping creation of directory: " + path + " (already exists).";
    return;
  }

  if (!dir.mkpath(".")) {
    QString error_msg = QString(
      "Failed to create directory \"%1\". Do you have sufficient permissions?").arg(path);
    qCritical() << error_msg;
    throw std::runtime_error(error_msg.toStdString());
  }
}

void write_file(QString path, QString content)
{
  QFile file(path);
  if (!file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate)) {
    QString error_msg = QString("Cannot open file %1 for writing: " + file.errorString()).arg(path);
    qCritical() << error_msg;
    throw std::runtime_error(error_msg.toStdString());
  }
  QTextStream out(&file);
  out << content;
  file.close();
}

QString read_file(QString path)
{
  QFile file(path);
  if (!file.open(QFile::ReadOnly | QFile::Text)) {
    QString error_msg = QString("Cannot open file %1 for reading: " + file.errorString()).arg(path);
    qCritical() << error_msg;
    throw std::runtime_error(error_msg.toStdString());
  }
  QTextStream in(&file);
  QString content = in.readAll();
  file.close();
  return content;
}

void append_to_file_before(QString file_path, QString lines_to_append, QString append_before_text)
{
  QString content = read_file(file_path);

  // Find the position to insert the lines before 'append_before_text'
  int index = content.indexOf(append_before_text);
  if (index != -1) {
    content.insert(index, lines_to_append);
  } else {
    QString error_msg = "Line " + append_before_text + " not found in the file " + file_path;
    throw std::runtime_error(error_msg.toStdString());
  }

  write_file(file_path, content);
}

void append_to_file(QString file_path, QString lines_to_append, QString append_after_text)
{
  QString content = read_file(file_path);

  // Find the position of the 'append_after_text'
  int index = content.indexOf(append_after_text);
  if (index != -1) {
    // Move the index to the end of the found text
    index += append_after_text.length();

    // Insert the new lines after the found text
    content.insert(index, lines_to_append);
  } else {
    QString error_msg = "Line " + append_after_text + " not found in the file " + file_path;
    throw std::runtime_error(error_msg.toStdString());
  }

  write_file(file_path, content);
}
