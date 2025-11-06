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

#include "turtle_nest/package_generators/python_package_generator.h"
#include "turtle_nest/file_utils.h"
#include <QDir>


void PythonPackageGenerator::create_package_impl(PackageInfo pkg_info){
  QStringList command = create_command("ament_python", pkg_info);
  run_command(command, pkg_info);
}

void PythonPackageGenerator::add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config){
  modify_setup_py(package_path, create_launch, create_config);
}

void modify_setup_py(QString package_path, bool create_launch, bool create_config)
{
  QString setup_py_path = QDir(package_path).filePath("setup.py");
  QString append_after = "('share/' + package_name, ['package.xml']),";

  if (create_launch) {
    QString lines_to_append =
        "\n        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),";
    append_to_file(setup_py_path, lines_to_append, append_after);
  }

  if (create_config) {
    QString config_content =
        "\n        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),";
    append_to_file(setup_py_path, config_content, append_after);
  }

  // Add imports if either launch or config was appended
  if (create_launch || create_config) {
    QString imports("import os\n"
                    "from glob import glob\n");
    append_to_file_before(setup_py_path, imports, "from setuptools import");
  }
}
