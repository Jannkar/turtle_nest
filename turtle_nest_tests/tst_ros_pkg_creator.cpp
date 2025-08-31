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
#include <turtle_nest/package_generators/base_package_generator.h>
#include "turtle_nest/package_generators/package_generator_factory.h"
#include "turtle_nest/rospkgcreator.h"
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


// Test the package creation with default parameters
TEST(ros_pkg_creator, create_pkg_defaults) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP);
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.workspace_path + "/package_name/CMakeLists.txt"));
}

// Test the package creation with all the parameters
TEST(ros_pkg_creator, create_pkg_all_values) {
  QTemporaryDir temp_dir;
  QString workspace_path = temp_dir.path() + "/ros2_ws";
  QString package_path = workspace_path + "/src/package_name";
  QString xml_path = package_path + "/package.xml";
  RosPkgCreator pkg_creator(workspace_path + "/src", "package_name", CPP_AND_PYTHON);
  pkg_creator.description = "multi-line test \n description";
  pkg_creator.maintainer_name = "Test Maintainer";
  pkg_creator.maintainer_email = "maintainer@admin.com";
  pkg_creator.node_name = "cpp_node";
  pkg_creator.node_type = NodeType::CPP_NODE;
  pkg_creator.launch_name = "demo_launch";
  pkg_creator.license = "Apache-2.0";
  pkg_creator.create_package();
  pkg_creator.build_package();

  ASSERT_TRUE(file_exists(package_path + "/CMakeLists.txt"));
  ASSERT_TRUE(file_exists(package_path + "/launch/demo_launch.py"));
  ASSERT_TRUE(file_exists(package_path + "/src/cpp_node.cpp"));
  ASSERT_TRUE(string_exists_in_file((package_path + "/LICENSE"), "Apache License"));
  ASSERT_EQ("Test Maintainer", read_xml_tag(xml_path, "maintainer"));
  ASSERT_EQ("multi-line test \n description", read_xml_tag(xml_path, "description"));
  ASSERT_EQ("maintainer@admin.com", read_xml_tag(xml_path, "maintainer", "email"));

  QString output = run_command("ros2 launch package_name demo_launch.py",
    {cpp_node_text}, pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(cpp_node_text));
}

// Try to create a package to destination the user doesn't have access rights
TEST(ros_pkg_creator, workspace_creation_fails) {
  RosPkgCreator pkg_creator("/root/no_permission", "package_name", CPP);
  EXPECT_THROW({
    pkg_creator.create_package();
  }, std::runtime_error);
}

// Create a C++ package without a Node
TEST(ros_pkg_creator, cpp_no_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP);
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(dir_is_empty(pkg_creator.package_path + "/src"));
}

// Create a C++ package with a Node
TEST(ros_pkg_creator, cpp_with_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP);
  pkg_creator.node_name = "cpp_node";
  pkg_creator.node_type = NodeType::CPP_NODE;
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));

  QString output = run_command("ros2 launch package_name test_launch.py", {cpp_node_text},
  pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(cpp_node_text));
}

// Create a C++ package with a Composable node, including params.
// Cannot be tested with other node types with ros2 run, as we need launch file
// to load the node in container.
TEST(ros_pkg_creator, cpp_composable_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP);
  pkg_creator.node_name = "cpp_node";
  pkg_creator.node_type = NodeType::CPP_COMPOSABLE_NODE;
  pkg_creator.launch_name = "test_launch";
  pkg_creator.params_file_name = "test_params";
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));

  QString container_text = "Loaded node '/cpp_node' in container '/cpp_node_container'";
  QString output = run_command("ros2 launch package_name test_launch.py", {cpp_node_text, param_text, container_text},
                               pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(cpp_node_text));
  ASSERT_TRUE(output.contains(param_text));
  ASSERT_TRUE(output.contains(container_text));
}

// Create a Python package without a Node
TEST(ros_pkg_creator, python_no_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", PYTHON);
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
}


// Create a Python package with a Node
TEST(ros_pkg_creator, python_with_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", PYTHON);
  pkg_creator.node_name = "python_node";
  pkg_creator.node_type = NodeType::PYTHON_NODE;
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/python_node.py"));
  ASSERT_TRUE(string_exists_in_file(
    (pkg_creator.package_path + "/package_name/python_node.py"),
      "Hello world from the Python node python_node"
  ));

  QString output = run_command("ros2 launch package_name test_launch.py", {python_node_text},
  pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(python_node_text));
}

// Create a CPP+Python package without Nodes
TEST(ros_pkg_creator, cpp_python_no_nodes) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(dir_is_empty(pkg_creator.package_path + "/src"));
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
}

// Create a CPP+Python package with CPP Node
TEST(ros_pkg_creator, cpp_python_with_cpp_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
  pkg_creator.node_name = "cpp_node";
  pkg_creator.node_type = NodeType::CPP_NODE;
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));

  QString output = run_command("ros2 launch package_name test_launch.py", {cpp_node_text},
  pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(cpp_node_text));
}

// Create a CPP+Python package with Python Node
TEST(ros_pkg_creator, cpp_python_with_python_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
  pkg_creator.node_name = "python_node";
  pkg_creator.node_type = NodeType::PYTHON_NODE;
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(dir_is_empty(pkg_creator.package_path + "/src"));
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/python_node.py"));
  ASSERT_TRUE(string_exists_in_file(
    (pkg_creator.package_path + "/package_name/python_node.py"),
      "Hello world from the Python node python_node"
  ));

  QString output = run_command("ros2 launch package_name test_launch.py", {python_node_text},
  pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(python_node_text));
}

// Create MSGS package
TEST(ros_pkg_creator, msgs_package){
  QString tmp_path = get_tmp_workspace_path();
  RosPkgCreator pkg_creator(tmp_path, "test_example_msgs", MSGS);
  pkg_creator.create_package();
  pkg_creator.build_package();

  // Create a Python package with a node that imports the new msgs package
  RosPkgCreator python_pkg_creator(tmp_path, "python_package", PYTHON);
  python_pkg_creator.node_name = "custom_message_import";
  python_pkg_creator.node_type = NodeType::PYTHON_NODE;
  python_pkg_creator.launch_name = "custom_message_import_launch";
  python_pkg_creator.create_package();
  QString node_path = QDir(python_pkg_creator.package_path).filePath("python_package/custom_message_import.py");
  QString fixture_path = QDir(FIXTURES_PATH).filePath("test_nodes/custom_message_import.py");
  write_file(node_path, read_file(fixture_path), true);
  python_pkg_creator.build_package();

  // In the Python node, we print the Example -message.
  // If it is printed correctly, the message importing was successful.
  QString expected_log = "test_example_msgs.msg.Example(example_data='example text')";
  QString output = run_command("ros2 launch python_package custom_message_import_launch.py", {expected_log}, python_pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(expected_log));
}

// /*
//  * TEST PARAMS FILE
// */

// Test that an empty params file is created when package with no Nodes is created (CPP package)
TEST(ros_pkg_creator, test_params_with_no_nodes_cpp) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "test_package", CPP);
  pkg_creator.launch_name = "test_launch";
  pkg_creator.params_file_name = "test_params";
  pkg_creator.create_package();
  pkg_creator.build_package();

  QString output = run_command(QString("ros2 launch test_package test_launch.py"),
    {"[INFO] [launch]:"}, pkg_creator.workspace_path);

  ASSERT_TRUE(!output.contains(param_text));

  QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(file_exists(params_path));
  ASSERT_EQ(read_file(params_path), "");

  ASSERT_TRUE(string_exists_in_file((pkg_creator.package_path + "/CMakeLists.txt"),
  "# Install config files"));
}

// Test that an empty params file is created when package with no Nodes is created (Python package)
TEST(ros_pkg_creator, test_params_with_no_nodes_python) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "test_package", PYTHON);
  pkg_creator.launch_name = "test_launch";
  pkg_creator.params_file_name = "test_params";
  pkg_creator.create_package();
  pkg_creator.build_package();

  QString output = run_command(QString("ros2 launch test_package test_launch.py"),
    {"[INFO] [launch]:"}, pkg_creator.workspace_path);

  ASSERT_TRUE(!output.contains(param_text));

  QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(file_exists(params_path));
  ASSERT_EQ(read_file(params_path), "");

  ASSERT_TRUE(string_exists_in_file((pkg_creator.package_path + "/setup.py"),
  "(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),"));
}

// Test the params file creation for all the Node types
TEST(ros_pkg_creator, test_node_params_creation){
  struct TestCase
  {
    NodeType node_type;
    QString node_name;
    QStringList outputs_to_wait;
  };

  QList<TestCase> test_cases = {
    {CPP_NODE, "cpp_node", {cpp_node_text, default_param_text}},
    {CPP_LIFECYCLE_NODE, "cpp_lifecycle_node", {cpp_lifecycle_node_text, default_param_text}},
    {PYTHON_NODE, "python_node", {python_node_text, default_param_text}},
    {PYTHON_LIFECYCLE_NODE, "python_lifecycle_node", {python_lifecycle_node_text, default_param_text}},
  };

  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "mixed_pkg", CPP_AND_PYTHON);
  pkg_creator.create_package();

  PackageInfo pkg_info;
  pkg_info.package_name = pkg_creator.package_name;
  pkg_info.package_path = pkg_creator.package_path;
  pkg_info.package_type = pkg_creator.build_type;

  std::unique_ptr<BasePackageGenerator> package_generator = create_package_generator(pkg_info.package_type);

  for (const auto & test_case : test_cases) {
    package_generator->add_node(
      {test_case.node_name, test_case.node_type, true},
      pkg_info.package_path,
      pkg_info.package_name);
  }
  pkg_creator.build_package();

  for (const auto & test_case : test_cases) {
    QString output = run_command(QString("ros2 run mixed_pkg %1").arg(test_case.node_name),
                                 test_case.outputs_to_wait, pkg_creator.workspace_path);

    for (const QString & expected : test_case.outputs_to_wait) {
      ASSERT_TRUE(output.contains(expected));
    }
  }
}

// Test that the params file is not created and the launch file is correctly populated when params_file_name is empty
TEST(ros_pkg_creator, test_params_not_set) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "test_package", CPP);
  pkg_creator.node_name = "cpp_node";
  pkg_creator.node_type = NodeType::CPP_NODE;
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();

  QString output = run_command(QString("ros2 launch test_package test_launch.py"),
  {cpp_node_text}, pkg_creator.workspace_path);

  ASSERT_TRUE(output.contains({cpp_node_text}));
  ASSERT_TRUE(!output.contains(param_text));

  QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(!file_exists(params_path));

  // Confirm that parameters are empty in the launch file
  ASSERT_TRUE(string_exists_in_file((pkg_creator.workspace_path + "/test_package" + "/launch/" +
  pkg_creator.launch_name + ".py"), "parameters=[]"));
}
