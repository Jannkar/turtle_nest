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

#include "turtle_nest/modify_existing_pkg.h"
#include "turtle_nest/node_type_enum.h"
#include "turtle_nest/node_generators/node_generator_factory.h"
#include "turtle_nest/packageinfo.h"
#include "turtle_nest/file_utils.h"
#include <QDebug>
#include <QDirIterator>
#include <QDesktopServices>
#include <QFile>
#include <QUrl>
#include <QRegularExpression>
#include "turtle_nest/node_generators/base_node_generator.h"
#include <turtle_nest/node_generators/cpp_node_generator.h>
#include <turtle_nest/node_generators/python_node_generator.h>
#include <turtle_nest/node_generators/mixed_cpp_python_node_generator.h>


std::map<QString, PackageInfo> list_packages(QString workspace_path)
{
  std::map<QString, PackageInfo> packages;

  // Block fetching packages in the very root.
  if (workspace_path == "" || workspace_path == "/") {
    return packages;
  }

  QDirIterator it(workspace_path, QStringList() << "package.xml", QDir::Files,
    QDirIterator::Subdirectories);
  while (it.hasNext()) {
    QString package_xml_path = it.next();
    QFileInfo fileInfo(package_xml_path);
    QDir package_dir = fileInfo.dir();     // Get the directory containing the package.xml
    if (!is_src_package(package_dir)) {
      continue;
    }

    PackageInfo pkg_info;
    pkg_info.workspace_path = workspace_path;
    pkg_info.package_name = read_xml_tag(package_xml_path, "name");
    pkg_info.package_path = package_dir.path();

    pkg_info.package_type = get_package_build_type(pkg_info.package_path, pkg_info.package_name);

    if (pkg_info.package_type == UNKNOWN) {
      qWarning() << "Unrecognized package type for" << pkg_info.package_path;
      continue;
    }

    pkg_info.description = clean_description(read_xml_tag(package_xml_path, "description"));
    pkg_info.maintainer = read_xml_tag(package_xml_path, "maintainer");
    pkg_info.version = read_xml_tag(package_xml_path, "version");
    pkg_info.license = read_xml_tag(package_xml_path, "license");
    packages[pkg_info.package_name.toLower()] = pkg_info;
  }

  return packages;
}

QStringList list_executables(const QString workspace_path, const QString package_name)
{
  QString workspace = get_workspace_path_without_src(workspace_path);

  QString install_path = QDir(workspace).filePath("install/" + package_name);
  QString base_path = QDir(install_path).filePath("lib/" + package_name);
  QStringList executables;
  QDirIterator it(base_path, QDir::Files | QDir::Executable, QDirIterator::Subdirectories);

  QDir dir(install_path);
  if (!dir.exists()) {
    std::string error_msg = "Path " + base_path.toStdString() + " doesn't exist.";
    throw std::runtime_error(error_msg);
  }
  while (it.hasNext()) {
    QString path = it.next();
    QFileInfo file_info(path);
    if (file_info.isExecutable()) {
      executables.append(file_info.fileName());
    }
  }
  executables.sort();
  return executables;
}


QStringList list_files(QString path)
{
  QStringList files;
  QDirIterator it(path, QDir::Files, QDirIterator::Subdirectories);

  while (it.hasNext()) {
    files.append(QFileInfo(it.next()).fileName());
  }
  return files;
}


void add_node(QString node_name, NodeType node_type, PackageInfo pkg_info)
{
  std::unique_ptr<BaseNodeGenerator> node_generator = create_node_generator(
    pkg_info.package_type);

  NodeOptions node_options{
    node_name,
    node_type,
    false, // add_params
  };

  // Node generation step will throw a runtime error if the node already exists.
  node_generator->add_node(node_options, pkg_info.package_path, pkg_info.package_name);
}

bool is_src_package(QDir dir)
{
  // Make sure we list only the src packages, not the built ones by
  // checking that CMakeLists.txt or setup.py exists
  return QFile::exists(dir.filePath("CMakeLists.txt")) || QFile::exists(dir.filePath("setup.py"));
}

bool is_python_package(QString package_path)
{
  return QFile::exists(QDir(package_path).filePath("setup.py"));
}

bool is_cpp_package(QString package_path)
{
  return QFile::exists(QDir(package_path).filePath("CMakeLists.txt"));
}

bool is_mixed_package(QString package_path)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");
  if (!QFile::exists(cmakelists_path)) {
    return false;
  }
  QString cmakelists_contents = read_file(cmakelists_path);

  // If Python files are installed in any other destination than to
  // /<package_name>/<package_name>, don't consider this as an ament_python
  // package, as the node creation is not supported.
  if (cmakelists_contents.contains(QString("\nament_python_install_package(${PROJECT_NAME})"))) {
    return true;
  }
  return false;
}

bool is_msgs_package(QString package_name)
{
  return package_name.endsWith("_msgs");
}


BuildType get_package_build_type(QString package_path, QString package_name)
{

  if (is_msgs_package(package_name)) {
    return MSGS;
  } else if (is_python_package(package_path)) {
    return PYTHON;
  } else if (is_mixed_package(package_path)) {
    return CPP_AND_PYTHON;
  } else if (is_cpp_package(package_path)) {
    return CPP;
  } else {
    return UNKNOWN;
  }
}


QString clean_description(const QString & input)
{
  // Package descriptions usually are "pretty-formatted", to be on the same
  // line as the <description> tag. This leads to extra whitespaces. Remove
  // the extra whitespaces, while maintaining the paragraph structure.
  QStringList paragraphs;
  static const QRegularExpression paragraphSeparator("\\n\\s*\\n");
  for (const QString & block : input.split(paragraphSeparator)) {
    QString line = block.simplified();
    paragraphs << line;
  }
  return paragraphs.join("\n\n");
}
