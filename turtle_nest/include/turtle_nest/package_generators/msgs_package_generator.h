#pragma once

#include <turtle_nest/package_generators/base_package_generator.h>

class MsgsPackageGenerator : public BasePackageGenerator
{
public:
  MsgsPackageGenerator() = default;
private:
  void create_package_impl(PackageInfo pkg_info) override;
  void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config) override;
};


void add_msgs_to_cmakelists(QString package_path);
void create_msgs_files(QString package_path);
QString get_msgs_cmake_addition();
QString get_example_msg_contents();
