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

#include "turtle_nest/package_generators/create_package.h"
#include "turtle_nest/package_generators/cpp_package_generator.h"
#include "turtle_nest/package_generators/mixed_package_generator.h"
#include "turtle_nest/package_generators/msgs_package_generator.h"
#include "turtle_nest/package_generators/python_package_generator.h"


// Create package using the appropriate package generator
void create_package(PackageInfo pkg_info)
{
  auto generator = get_package_generator(pkg_info.package_type);
  generator->create_package(pkg_info);
}

// Create launch and params files using the appropriate package generator
void create_launch_and_params(PackageInfo pkg_info, QString launch_name, QString params_file_name, QString node_name, bool composable_launch){
  auto generator = get_package_generator(pkg_info.package_type);
  generator->create_launch_and_params(pkg_info.package_path, pkg_info.package_name, launch_name, params_file_name, node_name, composable_launch);
}

std::unique_ptr<BasePackageGenerator> get_package_generator(BuildType package_type)
{
  if (package_type == BuildType::CPP) {
    return std::make_unique<CppPackageGenerator>();
  } else if (package_type == BuildType::PYTHON) {
    return std::make_unique<PythonPackageGenerator>();
  } else if (package_type == BuildType::CPP_AND_PYTHON) {
    return std::make_unique<MixedPackageGenerator>();
  } else if (package_type == BuildType::MSGS){
    return std::make_unique<MsgsPackageGenerator>();
  }else {
    throw std::runtime_error("Unknown package type");
  }
}
