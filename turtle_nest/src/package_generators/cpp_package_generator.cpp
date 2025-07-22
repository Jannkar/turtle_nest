/* ------------------------------------------------------------------
 * Copyright 2025 Janne Karttunen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------
*/

#include "turtle_nest/package_generators/cpp_package_generator.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/string_tools.h"
#include "turtle_nest/package_xml_tools.h"

void CppPackageGenerator::add_node(QString node_name, NodeType node_type, QString package_path, QString /*package_name*/){
  if (node_type == CPP_NODE) {
    generate_cpp_node(package_path, node_name, false);
    add_node_to_cmakelists(package_path, node_name);
  }
  else {
    throw std::runtime_error(
      QString("Unsupported node type: %1")
          .arg(node_type)
          .toStdString()
      );
  }
}


void generate_cpp_node(
    QString package_path, QString node_name, bool create_config,
    bool overwrite_existing)
{
  QString params_block =
      !create_config ? "" :
          R"(
    this->declare_parameter<std::string>("example_param", "default_value");
    std::string example_param = this->get_parameter("example_param").as_string();
    RCLCPP_INFO(
      this->get_logger(), "Declared parameter 'example_param'. Value: %s", example_param.c_str());
)";
  // Uncrustify considers this as a single line. Skip.
  /* *INDENT-OFF* */

  QString content = QString(R"(#include "rclcpp/rclcpp.hpp"

class %2 : public rclcpp::Node
{
public:
  %2()
  : Node("%1")
  {%3
    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "%1");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<%2>());
  rclcpp::shutdown();
  return 0;
}
)").arg(node_name, to_camel_case(node_name), params_block);
  /* *INDENT-ON* */
  write_file(QDir(package_path).filePath("src/" + node_name + ".cpp"), content, overwrite_existing);
  add_rclcpp_dependency_to_package_xml(package_path);
}


void add_node_to_cmakelists(QString package_path, QString node_name)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";

  add_dependency_to_cmakelists("rclcpp", cmakelists_path);

  QString content(
      R"(# Add node %1
add_executable(%1 src/%1.cpp)

target_include_directories(%1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  %1
  "rclcpp"
)

install(TARGETS %1
  DESTINATION lib/${PROJECT_NAME})

)");
  content = content.arg(node_name);
  append_to_file_before(cmakelists_path, content, append_before_text);
}

void add_dependency_to_cmakelists(QString dependency, QString cmakelists_path)
{
  QString target_string = QString("find_package(%1 REQUIRED)").arg(dependency);
  QString cmakelists_contents = read_file(cmakelists_path);

  // Return if dependency already exists
  if (cmakelists_contents.contains(target_string)) {
    return;
  }

  QString append_before_text = "ament_package()";
  append_to_file_before(cmakelists_path, target_string + "\n\n", append_before_text);
}


void add_rclcpp_dependency_to_package_xml(QString package_path)
{
  PackageXMLEditor xml_editor = PackageXMLEditor(package_path);
  bool has_depend = xml_editor.has_dependency("rclcpp", DependencyType::DEPEND);
  if (has_depend) {
    qDebug() << "Already has dependency rclcpp!";
    return;
  }

  bool has_exec_depend = xml_editor.has_dependency("rclcpp", DependencyType::EXEC_DEPEND);
  bool has_build_depend = xml_editor.has_dependency("rclcpp", DependencyType::BUILD_DEPEND);

  if (has_exec_depend && has_build_depend) {
    return;
  } else if (has_exec_depend && !has_build_depend) {
    xml_editor.add_dependency("rclcpp", DependencyType::BUILD_DEPEND);
  } else if (!has_exec_depend && has_build_depend) {
    xml_editor.add_dependency("rclcpp", DependencyType::EXEC_DEPEND);
  } else {
    xml_editor.add_dependency("rclcpp", DependencyType::DEPEND);
  }
}
