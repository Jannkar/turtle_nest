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

#include "turtle_nest/node_generators/cpp_node_generator.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/string_tools.h"
#include "turtle_nest/package_xml_tools.h"

void CppNodeGenerator::add_node(
  NodeOptions node_options, QString package_path,
  QString package_name)
{
  if (node_options.node_type == CPP_NODE) {
    generate_cpp_node(package_path, node_options.node_name, node_options.add_params);
    add_cpp_dependency(package_path, "rclcpp");
    add_node_to_cmakelists(package_path, node_options.node_name);
  } else if (node_options.node_type == CPP_LIFECYCLE_NODE) {
    generate_lifecycle_cpp_node(package_path, node_options);
    add_cpp_dependency(package_path, "rclcpp");
    add_cpp_dependency(package_path, "rclcpp_lifecycle");
    add_lifecycle_node_to_cmakelists(package_path, node_options.node_name);
  } else if (node_options.node_type == CPP_COMPOSABLE_NODE) {
    generate_composable_node(package_path, package_name, node_options);
    add_cpp_dependency(package_path, "rclcpp");
    add_cpp_dependency(package_path, "rclcpp_components");
    add_composable_node_to_cmakelists(package_path, package_name, node_options.node_name);
  } else {
    BaseNodeGenerator::add_node(node_options, package_path, package_name);
  }
}

void add_cpp_dependency(QString package_path, QString dependency)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");

  add_dependency_to_cmakelists(dependency, cmakelists_path);
  add_cpp_dependency_to_package_xml(package_path, dependency);
}

void generate_cpp_node(
  QString package_path, QString node_name, bool create_config,
  bool overwrite_existing)
{
  QString params_block = !create_config ? "" : get_params_block();

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
}


void add_node_to_cmakelists(QString package_path, QString node_name)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";

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

QString get_params_block()
{
  return QString(
    R"(
    this->declare_parameter<std::string>("example_param", "default_value");
    std::string example_param = this->get_parameter("example_param").as_string();
    RCLCPP_INFO(
      this->get_logger(), "Declared parameter 'example_param'. Value: %s", example_param.c_str());
)");
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

/**
 * @brief Adds a cpp-related dependency to package xml
 *
 * Adds a given dependency with a "depend" tag if it doesn't yet exist.
 * If the dependency exists with both "build_depend" and "exec_depend" tags,
 * skips the dependency adding. If only one of these tags exists, adds the missing tag.
*/
void add_cpp_dependency_to_package_xml(QString package_path, QString dependency)
{
  PackageXMLEditor xml_editor = PackageXMLEditor(package_path);
  bool has_depend = xml_editor.has_dependency(dependency, DependencyType::DEPEND);
  if (has_depend) {
    return;
  }

  bool has_exec_depend = xml_editor.has_dependency(dependency, DependencyType::EXEC_DEPEND);
  bool has_build_depend = xml_editor.has_dependency(dependency, DependencyType::BUILD_DEPEND);

  if (has_exec_depend && has_build_depend) {
    return;
  } else if (has_exec_depend && !has_build_depend) {
    xml_editor.add_dependency(dependency, DependencyType::BUILD_DEPEND);
  } else if (!has_exec_depend && has_build_depend) {
    xml_editor.add_dependency(dependency, DependencyType::EXEC_DEPEND);
  } else {
    xml_editor.add_dependency(dependency, DependencyType::DEPEND);
  }
}

void generate_lifecycle_cpp_node(QString package_path, NodeOptions node_options)
{
  QString params_block = node_options.add_params ? get_params_block() : "";

  // Uncrustify considers this as a single line. Skip.
  /* *INDENT-OFF* */

  QString content = QString(R"(#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class %2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  %2()
  : rclcpp_lifecycle::LifecycleNode("%1")
  {%3
    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "%1");
  }

  /* Available state transition callbacks for the lifecycle node. The ones that are
  not needed can be removed, as the default behavior already returns CallbackReturn::SUCCESS */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure() called.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate() called.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate() called.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_cleanup() called.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_shutdown() called.");
    return CallbackReturn::SUCCESS;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto lc_node = std::make_shared<%2>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(lc_node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
)").arg(node_options.node_name, to_camel_case(node_options.node_name), params_block);
  /* *INDENT-ON* */
  write_file(QDir(package_path).filePath("src/" + node_options.node_name + ".cpp"), content, false);
}

void add_lifecycle_node_to_cmakelists(QString package_path, QString node_name)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";

  QString content(
    R"(# Add lifecycle node %1
add_executable(%1 src/%1.cpp)

target_include_directories(%1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  %1
  "rclcpp"
  "rclcpp_lifecycle"
)

install(TARGETS %1
  DESTINATION lib/${PROJECT_NAME})

)");
  content = content.arg(node_name);
  append_to_file_before(cmakelists_path, content, append_before_text);
}

void generate_composable_node(QString package_path, QString package_name, NodeOptions node_options)
{
  QString params_block = node_options.add_params ? get_params_block() : "";

  // Uncrustify considers this as a single line. Skip.
  /* *INDENT-OFF* */

  QString content = QString(R"(#include "rclcpp/rclcpp.hpp"

namespace %4{

class %2 : public rclcpp::Node
{
public:
  %2(const rclcpp::NodeOptions & options)
  : Node("%1", options)
  {%3
    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "%1");
  }
};

} // namespace %4

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(%4::%2)

)").arg(node_options.node_name, to_camel_case(node_options.node_name), params_block, package_name);
  /* *INDENT-ON* */
  write_file(QDir(package_path).filePath("src/" + node_options.node_name + ".cpp"), content, false);
}

void add_composable_node_to_cmakelists(
  QString package_path, QString package_name,
  QString node_name)
{
  QString cmakelists_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";

  QString component_name = node_name + "_c";

  QString content(
    R"(# Add composable node %1
add_library(%2 SHARED src/%1.cpp)

ament_target_dependencies(
  %2
  "rclcpp"
  "rclcpp_components"
)

rclcpp_components_register_node(
  %2
  PLUGIN "%3::%4"
  EXECUTABLE %1
)

ament_export_targets(export_%2)
install(TARGETS %2
  EXPORT export_%2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

)");
  content = content.arg(node_name, component_name, package_name, to_camel_case(node_name));
  append_to_file_before(cmakelists_path, content, append_before_text);
}