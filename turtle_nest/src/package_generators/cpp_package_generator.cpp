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

#include "turtle_nest/package_generators/cpp_package_generator.h"
#include "turtle_nest/file_utils.h"
#include <QDir>


void CppPackageGenerator::create_package_impl(PackageInfo pkg_info){
  QStringList command = create_command("ament_cmake", pkg_info);
  run_command(command, pkg_info);
}


void CppPackageGenerator::add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config){
  modify_cmake_file(package_path, create_launch, create_config);
}

void modify_cmake_file(
    QString package_path, bool create_launch, bool create_config)
{
  QString c_make_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";
  if (create_launch) {
    QString launch_append =
        R"(# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

)";
    append_to_file_before(c_make_path, launch_append, append_before_text);
  }

  if (create_config) {
    append_to_file_before(c_make_path, cpp_config_installation_text(), append_before_text);
  }
}


QString cpp_config_installation_text(){
  return R"(# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

)";
}