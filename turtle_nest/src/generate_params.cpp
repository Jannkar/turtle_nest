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

#include "turtle_nest/generate_params.h"
#include "turtle_nest/file_utils.h"
#include <QDir>

void generate_params_file(
  QString package_path, QString params_file_name, QString node_name_cpp,
  QString node_name_python, BuildType build_type)
{
  QString params_file_dir = QDir(package_path).filePath("config");
  QString params_file_path = QDir(params_file_dir).filePath(params_file_name);
  create_directory(params_file_dir);
  write_file(params_file_path, get_params_content(node_name_cpp, node_name_python));

  if (build_type == PYTHON){
      add_params_to_setup_py(package_path);
  } // TODO: CPP
}

QString get_params_content(QString node_name_cpp, QString node_name_python)
{
  QString content = QString();
  if (node_name_cpp != "") {
    content = content + QString(R"(%1:
  ros__parameters:
    example_param: "abc"

)").arg(node_name_cpp);
  }

  if (node_name_python != "") {
    content = content + QString(R"(%1:
  ros__parameters:
    example_param: "abc"

)").arg(node_name_python);
  }
  return content;
}


void add_params_to_setup_py(QString package_path)
{
    QString setup_py_path = QDir(package_path).filePath("setup.py");
    QString file_content = read_file(setup_py_path);
    QString append_after = "('share/' + package_name, ['package.xml']),";

    QString config_declaration = "(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),";

    if (file_content.indexOf(config_declaration) == -1){
        QString config_content ="\n        " + config_declaration;
        append_to_file(setup_py_path, config_content, append_after);
    }

    // Add imports if needed
    QString os_import = "import os\n";
    QString glob_import = "from glob import glob\n";

    if (file_content.indexOf(os_import) == -1){
        append_to_file_before(setup_py_path, os_import, "from setuptools import");
    }

    if (file_content.indexOf(glob_import) == -1){
        append_to_file_before(setup_py_path, glob_import, "from setuptools import");
    }
}
