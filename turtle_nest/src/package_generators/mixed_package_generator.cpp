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

#include "turtle_nest/package_generators/mixed_package_generator.h"
#include "turtle_nest/node_generators/python_node_generator.h"
#include "turtle_nest/node_generators/mixed_cpp_python_node_generator.h"
#include "turtle_nest/package_generators/cpp_package_generator.h"


void MixedPackageGenerator::create_package_impl(PackageInfo pkg_info){
  QStringList command = create_command("ament_cmake", pkg_info);
  run_command(command, pkg_info);

  create_init_file(pkg_info.package_path, pkg_info.package_name);
  install_python_modules_in_cmakelists(pkg_info.package_path);
}

void MixedPackageGenerator::add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config){
  modify_cmake_file(package_path, create_launch, create_config);
}


