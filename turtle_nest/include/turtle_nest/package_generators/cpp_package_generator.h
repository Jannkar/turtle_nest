#pragma once

#include <turtle_nest/package_generators/base_package_generator.h>

class CppPackageGenerator : public BasePackageGenerator
{
public:
  CppPackageGenerator(PackageInfo pkg_info);
private:
  void create_package_impl() override;
  void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config) override;
};

void modify_cmake_file(
    QString package_path, bool create_launch, bool create_config);
