#pragma once

#include "turtle_nest/node_type_enum.h"
#include <turtle_nest/packageinfo.h>


class BasePackageGenerator
{
public:
  BasePackageGenerator() = default;
  void create_package(PackageInfo pkg_info);

  // We need to have both launch and params creation in one, since the cmakelists
  // append happens at the same time for both of them. Eventually, we want to move
  // at least the launch file creation into their own generators to support different
  // launch file formats.
  void create_launch_and_params(QString package_path, QString package_name, QString launch_name, QString params_file_name, QString node_name, bool composable_launch);

  QStringList create_command(QString type, PackageInfo pkg_info) const;
  void run_command(QStringList command, PackageInfo pkg_info) const;

private:
  // When creating new package types, override this function to implement the
  // package-specific creation. All the common creation happens in "create_package()" function,
  // which shouldn't be overridden.
  virtual void create_package_impl(PackageInfo pkg_info) = 0;
  virtual void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config) = 0;
};

void colcon_build(QString workspace_path, QStringList packages = QStringList());
