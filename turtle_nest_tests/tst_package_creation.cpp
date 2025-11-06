/* ------------------------------------------------------------------
 * Copyright 2024 Janne Karttunen
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

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>
#include <turtle_nest/node_generators/base_node_generator.h>
#include <turtle_nest/node_generators/node_generator_factory.h>
#include <turtle_nest/node_generators/python_node_generator.h>
#include "turtle_nest/modify_existing_pkg.h"
#include <turtle_nest/package_generators/cpp_package_generator.h>
#include <turtle_nest/package_generators/base_package_generator.h>
#include <turtle_nest/package_generators/create_package.h>
#include <turtle_nest/package_generators/mixed_package_generator.h>
#include <turtle_nest/package_generators/msgs_package_generator.h>
#include <turtle_nest/package_generators/python_package_generator.h>
#include "turtle_nest/file_utils.h"
#include "test_utils.h"
#include <QDebug>
#include <QTemporaryDir>
#include <QXmlStreamReader>
#include <QRegularExpression>
#include <QProcess>
#include <QTimer>
#include <QDateTime>
#include <QStringList>
#include <QElapsedTimer>
#include <QThread>


using namespace testing;


const QString python_node_text = "Hello world from the Python node python_node";
const QString python_lifecycle_node_text = "Hello world from the Python node python_lifecycle_node";
const QString cpp_node_text = "Hello world from the C++ node cpp_node";
const QString cpp_lifecycle_node_text = "Hello world from the C++ node cpp_lifecycle_node";
const QString param_text = "Declared parameter 'example_param'. Value: abc";
const QString default_param_text = "Declared parameter 'example_param'. Value: default_value";

QString get_expected_node_text(QString node_name, QString package_string){
  return "Hello world from the " + package_string + " node " + node_name;
}

std::vector<BuildType> get_build_types() {
  return {BuildType::CPP, BuildType::PYTHON, BuildType::CPP_AND_PYTHON, BuildType::MSGS};
}

QString build_type_to_string(BuildType build_type){
  switch (build_type) {
    case BuildType::CPP:
      return "cpp_package";
    case BuildType::PYTHON:
      return "python_package";
    case BuildType::CPP_AND_PYTHON:
      return "mixed_package";
    case BuildType::MSGS:
      return "msgs_package";
    default:
      throw std::runtime_error("Unknown package type");
  }
}

// Test the package creation with default, non-set parameters
TEST(package_creation, create_pkg_defaults) {
  for (BuildType build_type: get_build_types()) {
    PackageInfo pkg_info(build_type_to_string(build_type), get_tmp_package_dest(), build_type);
    create_package(pkg_info);
    colcon_build(pkg_info.package_destination, {pkg_info.package_name});

    // Check that the package was created properly
    QString xml_path = pkg_info.package_path + "/package.xml";
    ASSERT_EQ("user", read_xml_tag(xml_path, "maintainer"));
    ASSERT_EQ("todo@todo.todo", read_xml_tag(xml_path, "maintainer", "email"));
    ASSERT_EQ("TODO: License declaration", read_xml_tag(xml_path, "license"));
    ASSERT_EQ(pkg_info.package_name, read_xml_tag(xml_path, "name"));
    ASSERT_EQ("0.0.0", read_xml_tag(xml_path, "version"));
    ASSERT_EQ("TODO: Package description", read_xml_tag(xml_path, "description"));
  }
}

// Test the package creation with all the parameters
TEST(package_creation, create_pkg_all_values) {
  for (BuildType build_type: get_build_types()) {
    QString package_name = build_type_to_string(build_type);
    PackageInfo pkg_info(package_name, get_tmp_package_dest(), build_type);
    pkg_info.description = "multi-line test \n description";
    if (build_type == BuildType::PYTHON){
      //TODO: https://github.com/Jannkar/turtle_nest/issues/33
      pkg_info.description = "single-line description";
    }
    pkg_info.maintainer = "Test Maintainer";
    pkg_info.maintainer_email = "maintainer@admin.com";
    pkg_info.license = "Apache-2.0";
    create_package(pkg_info);
    colcon_build(pkg_info.package_destination, {pkg_info.package_name});

    // Check that the package was created properly
    QString xml_path = pkg_info.package_path + "/package.xml";
    ASSERT_EQ(pkg_info.maintainer, read_xml_tag(xml_path, "maintainer"));
    ASSERT_EQ(pkg_info.license, read_xml_tag(xml_path, "license"));
    ASSERT_EQ(pkg_info.package_name, read_xml_tag(xml_path, "name"));
    ASSERT_EQ("0.0.0", read_xml_tag(xml_path, "version"));
    ASSERT_EQ(pkg_info.description, read_xml_tag(xml_path, "description"));
    ASSERT_EQ(pkg_info.maintainer_email, read_xml_tag(xml_path, "maintainer", "email"));
  }
}

// Try to create a package to destination the user doesn't have access rights
TEST(package_creation, package_dir_creation_fails) {
  EXPECT_THROW({
    create_package({"package_name", "/root/no_permission", BuildType::CPP});
  }, std::runtime_error);
}

// C++ package tests, to explicitly check the created files
TEST(package_creation, cpp_package) {
  PackageInfo pkg_info("cpp_package", get_tmp_package_dest(), BuildType::CPP);
  create_package(pkg_info);
  colcon_build(pkg_info.package_destination, {pkg_info.package_name});
  ASSERT_TRUE(dir_is_empty(pkg_info.package_path + "/src"));
  ASSERT_TRUE(file_exists(pkg_info.package_path + "/CMakeLists.txt"));
}


// Create a C++ package with a Composable node, including params.
// Makes sure the launching as a component via launch file works
// correctly. ros2 run behavior is tested in later tests.
TEST(package_creation, cpp_composable_node) {
  PackageInfo pkg_info("cpp_package", get_tmp_package_dest(), BuildType::CPP);
  create_package(pkg_info);
  create_launch_and_params(
    pkg_info,
    "test_launch",
    "test_params",
    "cpp_node",
    true
  );
  add_node({"cpp_node", CPP_COMPOSABLE_NODE, true}, pkg_info);
  colcon_build(pkg_info.package_destination, {pkg_info.package_name});


  ASSERT_TRUE(file_exists(pkg_info.package_path + "/src/cpp_node.cpp"));

  QString container_text = "Loaded node '/cpp_node' in container '/cpp_node_container'";
  QString output = run_command("ros2 launch cpp_package test_launch.py", {cpp_node_text, param_text, container_text},
                               pkg_info.package_destination);
  ASSERT_TRUE(output.contains(cpp_node_text));
  ASSERT_TRUE(output.contains(param_text));
  ASSERT_TRUE(output.contains(container_text));
}

// Python package tests, to explicitly check the created files
TEST(package_creation, python_package) {
  PackageInfo pkg_info("python_package", get_tmp_package_dest(), BuildType::PYTHON);
  create_package(pkg_info);
  colcon_build(pkg_info.package_destination, {pkg_info.package_name});
  ASSERT_TRUE(file_exists(pkg_info.package_path + "/python_package/__init__.py"));
}

// Mixed package tests, to explicitly check the created files.
TEST(package_creation, mixed_package) {
  PackageInfo pkg_info("mixed_package", get_tmp_package_dest(), BuildType::CPP_AND_PYTHON);
  MixedPackageGenerator().create_package(pkg_info);
  colcon_build(pkg_info.package_destination);
  ASSERT_TRUE(dir_is_empty(pkg_info.package_path + "/src"));
  ASSERT_TRUE(file_exists(pkg_info.package_path + "/mixed_package/__init__.py"));
}

// MSGS package tests, to explicitly check the created files
TEST(package_creation, msgs_package){
  PackageInfo pkg_info("test_example_msgs", get_tmp_package_dest(), BuildType::MSGS);
  create_package(pkg_info);
  colcon_build(pkg_info.package_destination);

  // Create a Python package with a node that imports the new msgs package
  PackageInfo p_pkg_info("python_package", pkg_info.package_destination, BuildType::PYTHON);
  NodeOptions node_options({"custom_message_import", NodeType::PYTHON_NODE, false});
  create_package(p_pkg_info);
  create_launch_and_params(
    p_pkg_info,
    "custom_message_import_launch",
    "",
    node_options.node_name,
    false
  );
  add_node(node_options, p_pkg_info);
  QString node_path = QDir(p_pkg_info.package_path).filePath("python_package/custom_message_import.py");
  QString fixture_path = QDir(FIXTURES_PATH).filePath("test_nodes/custom_message_import.py");
  write_file(node_path, read_file(fixture_path), true);
  colcon_build(p_pkg_info.package_destination, {p_pkg_info.package_name});

  // In the Python node, we print the Example -message.
  // If it is printed correctly, the message importing was successful.
  QString expected_log = "test_example_msgs.msg.Example(example_data='example text')";
  QString output = run_command(
    "ros2 launch python_package custom_message_import_launch.py",
    {expected_log},
    p_pkg_info.package_destination);
  ASSERT_TRUE(output.contains(expected_log));
}

/*
 * TEST LAUNCH & PARAMS FILE
 */

// Test that an empty params file is created when package with no Nodes is created
TEST(package_creation, test_params_with_no_nodes_cpp) {
  QString package_name = "cpp_package";
  PackageInfo pkg_info(package_name, get_tmp_package_dest(), BuildType::CPP);
  create_package(pkg_info);
  create_launch_and_params(
    pkg_info,
    "test_launch",
    "test_params",
    "",
    false
  );
  colcon_build(pkg_info.package_destination, {pkg_info.package_name});

  QString output = run_command(QString("ros2 launch %1 %2.py").arg(pkg_info.package_name, "test_launch"),
  {"[INFO] [launch]:"}, pkg_info.package_destination);

  ASSERT_TRUE(!output.contains(param_text));

  QString params_path = QString(pkg_info.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(file_exists(params_path));
  ASSERT_EQ(read_file(params_path), "");

  // Pretty print
  qInfo().noquote() << read_file(pkg_info.package_path + "/CMakeLists.txt");
  ASSERT_TRUE(string_exists_in_file((pkg_info.package_path + "/CMakeLists.txt"),
    cpp_config_installation_text()));
}

// Test that an empty params file is created when package with no Nodes is created (Python package)
TEST(package_creation, test_params_with_no_nodes_python) {
  QString package_name = "python_package";
  PackageInfo pkg_info(package_name, get_tmp_package_dest(), BuildType::PYTHON);
  create_package(pkg_info);
  create_launch_and_params(
    pkg_info,
    "test_launch",
    "test_params",
    "",
    false
  );
  colcon_build(pkg_info.package_destination, {pkg_info.package_name});

  QString output = run_command(QString("ros2 launch %1 %2.py").arg(pkg_info.package_name, "test_launch"),
    {"[INFO] [launch]:"}, pkg_info.package_destination);

  ASSERT_TRUE(!output.contains(param_text));

  QString params_path = QString(pkg_info.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(file_exists(params_path));
  ASSERT_EQ(read_file(params_path), "");

  ASSERT_TRUE(string_exists_in_file((pkg_info.package_path + "/setup.py"),
  "(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),"));
}

// Test the params file creation for all the Node types
TEST(package_creation, test_node_params_creation){
  struct TestCase
  {
    NodeType node_type;
    QString node_name;
    QStringList outputs_to_wait;
  };

  QList<TestCase> test_cases = {
    {CPP_NODE, "cpp_node", {cpp_node_text, default_param_text}},
    {CPP_COMPOSABLE_NODE, "cpp_node_composable",
      {get_expected_node_text("cpp_node_composable", "C++"), default_param_text}
    },
    {CPP_LIFECYCLE_NODE, "cpp_lifecycle_node", {cpp_lifecycle_node_text, default_param_text}},
    {PYTHON_NODE, "python_node", {python_node_text, default_param_text}},
    {PYTHON_LIFECYCLE_NODE, "python_lifecycle_node", {python_lifecycle_node_text, default_param_text}},
  };

  PackageInfo pkg_info("mixed_pkg", get_tmp_package_dest(), BuildType::CPP_AND_PYTHON);
  create_package(pkg_info);

  for (const auto & test_case : test_cases) {
    add_node({test_case.node_name, test_case.node_type, true}, pkg_info);
  }
  colcon_build(pkg_info.package_destination, {pkg_info.package_name});

  for (const auto & test_case : test_cases) {
    QString output = run_command(QString("ros2 run mixed_pkg %1").arg(test_case.node_name),
                                 test_case.outputs_to_wait, pkg_info.package_destination);

    for (const QString & expected : test_case.outputs_to_wait) {
      ASSERT_TRUE(output.contains(expected));
    }
  }
}

// Test that the params file is not created and the launch file is correctly populated when params_file_name is empty
TEST(package_creation, test_params_not_set) {
  PackageInfo pkg_info("test_package", get_tmp_package_dest(), BuildType::CPP);
  create_package(pkg_info);
  NodeOptions node_options({"cpp_node", NodeType::CPP_NODE, false});
  create_launch_and_params(
    pkg_info,
    "test_launch",
    "",
    node_options.node_name,
    false
  );
  add_node(node_options, pkg_info);
  colcon_build(pkg_info.package_destination, {pkg_info.package_name});

  QString output = run_command(QString("ros2 launch test_package test_launch.py"),
  {cpp_node_text}, pkg_info.package_destination);

  ASSERT_TRUE(output.contains({cpp_node_text}));
  ASSERT_TRUE(!output.contains(param_text));

  QString params_path = QString(pkg_info.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(!file_exists(params_path));

  // Confirm that parameters are empty in the launch file
  ASSERT_TRUE(string_exists_in_file((pkg_info.package_path + "/launch/test_launch.py"),
  "parameters=[]"));
}
