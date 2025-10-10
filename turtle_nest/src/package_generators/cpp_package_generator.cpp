#include "turtle_nest/package_generators/cpp_package_generator.h"
#include "turtle_nest/file_utils.h"
#include <QDir>


void CppPackageGenerator::create_package_impl(){
  QStringList command = create_command("ament_cmake");
  run_command(command);
}


void add_launch_and_params_to_config_(QString package_path, bool create_launch, bool create_config){
  modify_cmake_file(package_path, create_launch, create_config);
}


void modify_cmake_file(
    QString package_path, bool create_launch, bool create_config)
{
  QString c_make_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";
  if (create_launch) {
    QString launch_append =
        R"(# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

)";
    append_to_file_before(c_make_path, launch_append, append_before_text);
  }

  if (create_config) {
    QString config_append =
        R"(# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

)";
    append_to_file_before(c_make_path, config_append, append_before_text);
  }
}
