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


const QString python_node_text = "[python_node]: Hello world from the Python node python_node";
const QString python_param_text = "[python_node]: Declared parameter 'example_param'. Value: abc";
const QString cpp_node_text = "[cpp_node]: Hello world from the C++ node cpp_node";
const QString cpp_param_text = "[cpp_node]: Declared parameter 'example_param'. Value: abc";


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
  pkg_creator.node_name_cpp = "cpp_node";
  pkg_creator.node_name_python = "python_node";
  pkg_creator.launch_name = "demo_launch";
  pkg_creator.license = "Apache-2.0";
  pkg_creator.create_package();
  pkg_creator.build_package();

  ASSERT_TRUE(file_exists(package_path + "/CMakeLists.txt"));
  ASSERT_TRUE(file_exists(package_path + "/launch/demo_launch.py"));
  ASSERT_TRUE(file_exists(package_path + "/package_name/python_node.py"));
  ASSERT_TRUE(file_exists(package_path + "/package_name/__init__.py"));
  ASSERT_TRUE(file_exists(package_path + "/src/cpp_node.cpp"));
  ASSERT_TRUE(string_exists_in_file((package_path + "/LICENSE"), "Apache License"));
  ASSERT_EQ("Test Maintainer", read_xml_tag(xml_path, "maintainer"));
  ASSERT_EQ("multi-line test \n description", read_xml_tag(xml_path, "description"));
  ASSERT_EQ("maintainer@admin.com", read_xml_tag(xml_path, "maintainer", "email"));

  QString output = run_command("ros2 launch package_name demo_launch.py",
    {python_node_text, cpp_node_text}, pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(python_node_text));
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
  pkg_creator.node_name_cpp = "cpp_node";
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));

  QString output = run_command("ros2 launch package_name test_launch.py", {cpp_node_text},
  pkg_creator.workspace_path);
  ASSERT_TRUE(output.contains(cpp_node_text));
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
  pkg_creator.node_name_python = "python_node";
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
  pkg_creator.node_name_cpp = "cpp_node";
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
  pkg_creator.node_name_python = "python_node";
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


// Create a CPP+Python package with both Nodes
TEST(ros_pkg_creator, cpp_python_with_both_nodes) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
  pkg_creator.node_name_cpp = "cpp_node";
  pkg_creator.node_name_python = "python_node";
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
  ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/python_node.py"));
  ASSERT_TRUE(string_exists_in_file(
    (pkg_creator.package_path + "/package_name/python_node.py"),
      "Hello world from the Python node python_node"
  ));

  QStringList outputs_to_wait = {python_node_text, cpp_node_text};
  QString output = run_command(QString("ros2 launch package_name test_launch.py"),
  outputs_to_wait, pkg_creator.workspace_path);
  for (const QString & expected : outputs_to_wait) {
  ASSERT_TRUE(output.contains(expected));
  }
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

  ASSERT_TRUE(!output.contains(python_param_text));
  ASSERT_TRUE(!output.contains(cpp_param_text));

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

  ASSERT_TRUE(!output.contains(python_param_text));
  ASSERT_TRUE(!output.contains(cpp_param_text));

  QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(file_exists(params_path));
  ASSERT_EQ(read_file(params_path), "");

  ASSERT_TRUE(string_exists_in_file((pkg_creator.package_path + "/setup.py"),
  "(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),"));
}

// Test the params file creation with both Python Package and Node
TEST(ros_pkg_creator, test_params_with_python_node) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "test_package", PYTHON);
  pkg_creator.node_name_python = "python_node";
  pkg_creator.launch_name = "test_launch";
  pkg_creator.params_file_name = "test_params";
  pkg_creator.create_package();
  pkg_creator.build_package();

  QStringList outputs_to_wait = {python_node_text, python_param_text};
  QString output = run_command(QString("ros2 launch test_package test_launch.py"),
  outputs_to_wait, pkg_creator.workspace_path);

  for (const QString & expected : outputs_to_wait) {
    ASSERT_TRUE(output.contains(expected));
  }

  ASSERT_TRUE(!output.contains(cpp_node_text));
  ASSERT_TRUE(!output.contains(cpp_param_text));
}


// Test the params file creation with both CPP and Python Nodes
TEST(ros_pkg_creator, test_params_with_both_nodes) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "test_package", CPP_AND_PYTHON);
  pkg_creator.node_name_cpp = "cpp_node";
  pkg_creator.node_name_python = "python_node";
  pkg_creator.launch_name = "test_launch";
  pkg_creator.params_file_name = "test_params";
  pkg_creator.create_package();
  pkg_creator.build_package();

  QStringList outputs_to_wait = {python_node_text, cpp_node_text, python_param_text,
  cpp_param_text};
  QString output = run_command(QString("ros2 launch test_package test_launch.py"),
  outputs_to_wait, pkg_creator.workspace_path);

  for (const QString & expected : outputs_to_wait) {
    ASSERT_TRUE(output.contains(expected));
  }
}

// Test that the params file is not created and the launch file is correctly populated when params_file_name is empty
TEST(ros_pkg_creator, test_params_not_set) {
  RosPkgCreator pkg_creator(get_tmp_workspace_path(), "test_package", CPP_AND_PYTHON);
  pkg_creator.node_name_cpp = "cpp_node";
  pkg_creator.node_name_python = "python_node";
  pkg_creator.launch_name = "test_launch";
  pkg_creator.create_package();
  pkg_creator.build_package();

  QStringList outputs_to_wait = {python_node_text, cpp_node_text};

  QString output = run_command(QString("ros2 launch test_package test_launch.py"),
  outputs_to_wait, pkg_creator.workspace_path);

  for (const QString & expected : outputs_to_wait) {
    ASSERT_TRUE(output.contains(expected));
  }

  ASSERT_TRUE(!output.contains(python_param_text));
  ASSERT_TRUE(!output.contains(cpp_param_text));

  QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
  ASSERT_TRUE(!file_exists(params_path));

  // Confirm that parameters are empty in the launch file
  ASSERT_TRUE(string_exists_in_file((pkg_creator.workspace_path + "/test_package" + "/launch/" +
  pkg_creator.launch_name + ".py"), "parameters=[]"));
}
