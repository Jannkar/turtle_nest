#include <turtle_nest/packageinfo.h>

#pragma once

class BasePackageGenerator
{
public:
  BasePackageGenerator(PackageInfo pkg_info);
  void create_package();
  void create_node();

  // We need to have these both in one, since the cmakelists append happens around
  // at this point.
  void create_launch_and_params();

private:
  PackageInfo pkg_info_;
};

