#pragma once

#include "turtle_nest/node_type_enum.h"
#include <turtle_nest/packageinfo.h>


class BasePackageGenerator
{
public:
  BasePackageGenerator(PackageInfo pkg_info);
  void create_package();
  void create_node(QString node_name, NodeType node_type, bool create_config);

  // We need to have these both in one, since the cmakelists append happens around
  // at this point.
  void create_launch_and_params(QString launch_name, QString params_file_name, QString node_name, bool composable_launch);

  QStringList create_command(QString type) const;
  void run_command(QStringList command) const;
  PackageInfo pkg_info;

private:
  // When creating new package types, override this function to implement the
  // package-specific creation. All the common creation happens in "create_package()" function,
  // which shouldn't be overridden.
  virtual void create_package_impl() = 0;
  virtual void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config) = 0;
};

