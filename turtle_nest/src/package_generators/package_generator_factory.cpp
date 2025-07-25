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

#include "turtle_nest/package_generators/package_generator_factory.h"
#include <turtle_nest/package_generators/cpp_package_generator.h>
#include <turtle_nest/package_generators/mixed_cpp_python_package_generator.h>
#include <turtle_nest/package_generators/python_package_generator.h>

std::unique_ptr<BasePackageGenerator> create_package_generator(BuildType package_type){
  if (package_type == CPP) {
    return std::make_unique<CppPackageGenerator>();
  } else if (package_type == PYTHON) {
    return std::make_unique<PythonPackageGenerator>();
  } else if (package_type == CPP_AND_PYTHON) {
    return std::make_unique<MixedCppPythonPackageGenerator>();
  } else {
    return std::make_unique<BasePackageGenerator>();
  }
}
