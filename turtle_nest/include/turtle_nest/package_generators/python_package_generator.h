#pragma once

#include <turtle_nest/package_generators/base_package_generator.h>

class PythonPackageGenerator : public BasePackageGenerator
{
public:
  PythonPackageGenerator() = default;
private:
  void create_package_impl(PackageInfo pkg_info) override;
  void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config) override;
};

void modify_setup_py(QString package_path, bool create_launch, bool create_config);

