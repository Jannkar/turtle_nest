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

#pragma once

#include <turtle_nest/package_generators/base_package_generator.h>

class PythonPackageGenerator: public BasePackageGenerator
{
public:
  PythonPackageGenerator() = default;

private:
  void create_package_impl(PackageInfo pkg_info) override;
  void add_launch_and_params_to_config_(
    QString package_path, bool create_launch,
    bool create_config) override;
};

void modify_setup_py(QString package_path, bool create_launch, bool create_config);
