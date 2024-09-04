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

#include "turtle_nest/generate_setup_py.h"
#include "turtle_nest/file_utils.h"
#include <QDir>

void modify_setup_py(QString package_path, bool create_launch, bool create_config)
{
  QString setup_py_path = QDir(package_path).filePath("setup.py");
  QString append_after = "('share/' + package_name, ['package.xml']),";

  if (create_config) {
    QString config_content =
      "\n        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),";
    append_to_file(setup_py_path, config_content, append_after);
  }

  if (create_launch) {
    QString lines_to_append =
      "\n        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),";
    append_to_file(setup_py_path, lines_to_append, append_after);
  }

  // Add imports if either launch or config was appended
  if (create_launch || create_config) {
    QString imports("import os\n"
      "from glob import glob\n");
    append_to_file_before(setup_py_path, imports, "from setuptools import");
  }
}
