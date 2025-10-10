#include "turtle_nest/package_generators/mixed_package_generator.h"
#include "turtle_nest/node_generators/python_node_generator.h"
#include "turtle_nest/node_generators/mixed_cpp_python_node_generator.h"
#include "turtle_nest/package_generators/cpp_package_generator.h"


void MixedPackageGenerator::create_package_impl(){
  QStringList command = create_command("ament_cmake");
  run_command(command);

  create_init_file(pkg_info.package_path, pkg_info.package_name);
  install_python_modules_in_cmakelists(pkg_info.package_path);
}

void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config){
  modify_cmake_file(package_path, create_launch, create_config);
}


