#include "turtle_nest/package_generators/base_package_generator.h"
#include "turtle_nest/file_utils.h"


BasePackageGenerator::BasePackageGenerator(PackageInfo pkg_info)
  : pkg_info_(pkg_info)
{
  // ROS automatically populates the email from maintainer name, if email is not provided.
  // If the name contains invalid characters (or spaces) for email, the package building
  // will fail. This is why we need to set this default email instead of leaving it empty.
  if (pkg_info_.maintainer_email.isEmpty()){
    pkg_info_.maintainer_email = "todo@todo.todo";
  }

  if (pkg_info_.license.isEmpty()){
    pkg_info_.license = "TODO: License declaration";
  }
}


void BasePackageGenerator::create_package(){
  create_directory(pkg_info_.workspace_path);
}
