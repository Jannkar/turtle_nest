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
#include <unistd.h>
#include "turtle_nest/file_utils.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QXmlStreamReader>
#include <QCoreApplication>
#include <QStandardPaths>


void create_directory(QString path)
{
  QDir dir(path);
  if (dir.exists()) {
    qDebug() << "Skipping creation of directory: " + path + " (already exists).";
    return;
  }

  if (!dir.mkpath(".")) {
    QString error_msg = QString(
      "Failed to create directory \"%1\". Do you have sufficient permissions?").arg(path);
    qCritical() << error_msg;
    throw std::runtime_error(error_msg.toStdString());
  }
}

void write_file(QString path, QString content, bool overwrite)
{
  QFile file(path);
  QString directory_path = QFileInfo(file).absolutePath();
  create_directory(directory_path);

  // Raise an exception if file exists and overwrite is false
  if (file.exists() && !overwrite) {
      QString error_msg = QString("File %1 already exists!").arg(path);
      qCritical() << error_msg;
      throw std::runtime_error(error_msg.toStdString());
  }

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

  write_file(file_path, content, true);
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

  write_file(file_path, content, true);
}


QString read_xml_tag(const QString &filePath, const QString &tagName, const QString &attributeName) {
    QFile file(filePath);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Error opening file:" << file.errorString();
        return QString();
    }

    QXmlStreamReader xmlReader(&file);
    QString result = "";

    // Parse the XML file
    while (!xmlReader.atEnd() && !xmlReader.hasError()) {
        QXmlStreamReader::TokenType token = xmlReader.readNext();

        // If token is a StartElement and the tag name matches
        if (token == QXmlStreamReader::StartElement) {
            if (xmlReader.name() == tagName) {
                if (!attributeName.isEmpty() && xmlReader.attributes().hasAttribute(attributeName)) {
                    // If attribute is specified and exists, return the attribute value
                    result = xmlReader.attributes().value(attributeName).toString();
                } else {
                    // Otherwise, return the tag's text content
                    result = xmlReader.readElementText();
                }
                break; // Stop after finding the first matching tag or attribute
            }
        }
    }

    if (xmlReader.hasError()) {
        qWarning() << "Error reading XML file:" << xmlReader.errorString();
    }

    file.close();
    return result;
}

/**
 * @brief Returns the path of the current executable
**/
QString get_executable_dir() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    if (count == -1) return "";
    QString path = QString::fromUtf8(result, count);
    return QFileInfo(path).absolutePath();
}


QString get_workspace_path_with_src(QString path){
    if (path.isEmpty()){
        return "";
    }
    if (!path.endsWith('/')) {
        path += '/';
    }

    if (path.endsWith("src/")) {
        return path;
    } else {
        return path + "src/";
    }
}

QString get_workspace_path_without_src(QString path){
    if (path.isEmpty()){
        return "";
    }
    if (!path.endsWith('/')) {
        path += '/';
    }

    if (path.endsWith("src/")) {
        return path.chopped(QString("src/").length());
    } else {
        return path;
    }
}



