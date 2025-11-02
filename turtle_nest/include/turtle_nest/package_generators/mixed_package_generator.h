#pragma once

#include <turtle_nest/package_generators/base_package_generator.h>

class MixedPackageGenerator : public BasePackageGenerator
{
public:
  MixedPackageGenerator() = default;
private:
  void create_package_impl(PackageInfo pkg_info) override;
  void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config) override;
};

