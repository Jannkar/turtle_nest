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

#include "turtle_nest/file_utils.h"
#include "turtle_nest/modify_existing_pkg.h"
#include "turtle_nest/package_generators/create_package.h"
#include "test_utils.h"
#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>
#include <QDebug>
#include <QTemporaryDir>
#include <turtle_nest/package_xml_tools.h>

// Integration tests to verify modifying an existing packages


void execute_node_and_assert_success(PackageInfo pkg_info, QString node_name, NodeType node_type)
{
  QString node_type_str;
  if ((node_type == CPP_NODE) | (node_type == CPP_LIFECYCLE_NODE)){
    node_type_str = "C++";
  }else if ((node_type == PYTHON_NODE) | (node_type == PYTHON_LIFECYCLE_NODE)){
    node_type_str = "Python";
  }else{
    throw std::runtime_error("Invalid node type");
  }
  QString expected_output = QString("[%1]: Hello world from the %2 node %1").arg(node_name,
    node_type_str);
  QString output = run_command(
    QString("ros2 run %1 %2").arg(pkg_info.package_name, node_name),
    {expected_output},
    pkg_info.package_destination
  );

  ASSERT_TRUE(output.contains(expected_output));
}

TEST(modify_existing_pkg, list_packages) {
  struct TestCase
  {
    BuildType package_type;
    QString pkg_name;
  };

  // In alphabetical order by package name
  QList<TestCase> test_cases = {
    {BuildType::CPP, "cpp_package"},
    {BuildType::CPP_AND_PYTHON, "mixed"},
    {BuildType::PYTHON, "python_package"},
  };

  QString maintainer_name = "test_maintainer";
  QString description = "test_description";
  QString license = "test_license";
  QString version = "0.0.0";

  QTemporaryDir temp_dir;
  QString src_path = get_tmp_package_dest();
  QString workspace_path = get_workspace_path_without_src(src_path);

  for (TestCase & test_case : test_cases) {
    PackageInfo pkg_info(test_case.pkg_name, workspace_path, test_case.package_type);
    pkg_info.maintainer = maintainer_name;
    pkg_info.description = description;
    pkg_info.license = license;
    create_package(pkg_info);
  }

  std::map<QString, PackageInfo> packages = list_packages(workspace_path);
  int i = 0;
  for (const auto & [package_name, pkg_info] : packages) {
    ASSERT_EQ(pkg_info.package_name, test_cases[i].pkg_name);
    ASSERT_EQ(pkg_info.package_type, test_cases[i].package_type);
    ASSERT_EQ(pkg_info.maintainer, maintainer_name);
    ASSERT_EQ(pkg_info.description, description);
    ASSERT_EQ(pkg_info.version, version);
    ASSERT_EQ(pkg_info.license, license);
    ++i;
  }
}


// Test adding a new node to an existing ROS 2 package
TEST(modify_existing_pkg, add_node_to_existing_package) {
  struct TestCase
  {
    BuildType package_type;
    NodeType node_type;
    QString pkg_name;
  };

  QList<TestCase> test_cases = {
    {CPP, CPP_NODE, "cpp_package"},
    {CPP, CPP_LIFECYCLE_NODE, "cpp_lifecycle_package"},
    {PYTHON, PYTHON_NODE, "python_package"},
    {PYTHON, PYTHON_LIFECYCLE_NODE, "python_lifecycle_package"},
    {CPP_AND_PYTHON, CPP_NODE, "mixed_with_cpp_node"},
    {CPP_AND_PYTHON, CPP_LIFECYCLE_NODE, "mixed_with_cpp_lifecycle_node"},
    {CPP_AND_PYTHON, PYTHON_NODE, "mixed_with_python_node"},
    {CPP_AND_PYTHON, PYTHON_LIFECYCLE_NODE, "mixed_with_python_lifecycle_node"},
  };

  for (TestCase & test_case : test_cases) {
    PackageInfo pkg_info(test_case.pkg_name, get_tmp_package_dest(), test_case.package_type);
    create_package(pkg_info);
    add_node({"node_1", test_case.node_type}, pkg_info);
    add_node({"node_2", test_case.node_type}, pkg_info);
    colcon_build(pkg_info.package_destination, {pkg_info.package_name});

    execute_node_and_assert_success(pkg_info, "node_1", test_case.node_type);
    execute_node_and_assert_success(pkg_info, "node_2", test_case.node_type);

    // Test that dependencies were added correctly to package XML
    PackageXMLEditor xml_editor = PackageXMLEditor(pkg_info.package_path);
    switch (test_case.node_type) {
      case PYTHON_NODE:
        ASSERT_TRUE(xml_editor.has_dependency("rclpy", DependencyType::EXEC_DEPEND));
        break;
      case PYTHON_LIFECYCLE_NODE:
        ASSERT_TRUE(xml_editor.has_dependency("rclpy", DependencyType::EXEC_DEPEND));
        break;
      case CPP_NODE:
        ASSERT_TRUE(xml_editor.has_dependency("rclcpp", DependencyType::DEPEND));
        break;
      case CPP_LIFECYCLE_NODE:
        ASSERT_TRUE(xml_editor.has_dependency("rclcpp", DependencyType::DEPEND));
        ASSERT_TRUE(xml_editor.has_dependency("rclcpp_lifecycle", DependencyType::DEPEND));
        break;
      default:
        throw std::runtime_error("Invalid mode in switch");
    }
  }
}

// Trying to add another node with the same name raises an exception
TEST(modify_existing_pkg, try_adding_node_with_existing_name) {
  struct TestCase
  {
    BuildType package_type;
    NodeType node_type;
  };

  QList<TestCase> test_cases = {
    {CPP, CPP_NODE},
    {CPP, CPP_LIFECYCLE_NODE},
    {PYTHON, PYTHON_NODE},
    {PYTHON, PYTHON_LIFECYCLE_NODE},
    {CPP_AND_PYTHON, CPP_NODE},
    {CPP_AND_PYTHON, CPP_LIFECYCLE_NODE},
    {CPP_AND_PYTHON, PYTHON_NODE},
    {CPP_AND_PYTHON, PYTHON_LIFECYCLE_NODE},
  };

  for (TestCase & test_case : test_cases) {
    PackageInfo pkg_info("test_package", get_tmp_package_dest(), test_case.package_type);
    create_package(pkg_info);
    add_node({"node_1", test_case.node_type}, pkg_info);
    EXPECT_THROW(add_node({"node_1", test_case.node_type}, pkg_info), std::runtime_error);
  }
}

// Packages could have a name "test_project", even though the path of
// the package is .../src/project. Make sure that we add the node to
// the right place in these situations.
TEST(modify_existing_pkg, add_node_to_package_with_different_project_name) {
  PackageInfo pkg_info("test_package", get_tmp_package_dest(), CPP);
  create_package(pkg_info);

  QDir dir;
  dir.rename(pkg_info.package_path, pkg_info.package_destination + "/" + "package");

  QString workspace_path = get_workspace_path_without_src(pkg_info.package_destination);
  auto packages = list_packages(workspace_path);
  auto first_package_iter = packages.begin();
  auto first_package_info = first_package_iter->second;
  add_node({"node_1", CPP_NODE}, first_package_info);
  colcon_build(workspace_path, {pkg_info.package_name});

  execute_node_and_assert_success(pkg_info, "node_1", CPP_NODE);
}

// Exception is raised if setup.py is missing expected data-fields
TEST(modify_existing_pkg, add_python_node_invalid_setup_py) {
  QStringList fixture_files = {
    "setup_py_files/no_entry_points.setup.py",
    "setup_py_files/no_console_scripts.setup.py",
    "setup_py_files/empty.setup.py",
    "setup_py_files/no_setup.setup.py"
  };

  for (const auto & setup_py_file_name : fixture_files) {
    QString workspace_path = get_tmp_package_dest();
    PackageInfo pkg_info("test_package", get_tmp_package_dest(), PYTHON);
    create_package(pkg_info);

    QString setup_py_path = QDir(pkg_info.package_path).filePath("setup.py");
    QString faulty_setup_py_file = QString(QDir(FIXTURES_PATH).filePath(setup_py_file_name));
    QString faulty_content = read_file(faulty_setup_py_file);

    write_file(setup_py_path, faulty_content, true);
    EXPECT_THROW(add_node({"node_1", PYTHON_NODE}, pkg_info), std::runtime_error);

    // Double-check that there were no changes to the file
    QString current_content = read_file(setup_py_path);
    ASSERT_EQ(faulty_content, current_content);
  }
}

// Goes through the different XML scenarios for adding rclpy and rclcpp dependency
TEST(modify_existing_pkg, add_node_xml_tests) {
  // Test cases:
  // rclpy <exec_depend> already exists -> OK
  // rclpy <depend> already exists -> OK
  // rclpy no dependency yet -> Add exec_depend

  // rclcpp no dependency yet -> Add general <depend>
  // rclcpp depend only -> OK
  // rclcpp build and exec -> OK
  // rclcpp build only -> Add <exec_depend>
  // rclcpp exec only -> Add <build_depend>
  struct TestCase
  {
    BuildType build_type;
    NodeType node_type;
    QString package_xml_file;
    QString expected_result_xml_file;
  };

  QList<TestCase> test_cases = {
    {PYTHON, PYTHON_NODE, "rclpy_depend.package.xml", "rclpy_depend.package.xml"},     // Expect no changes
    {PYTHON, PYTHON_NODE, "rclpy_exec_depend.package.xml", "rclpy_exec_depend.package.xml"},     // Expect no changes
    {PYTHON, PYTHON_NODE, "rclpy_empty_depend.package.xml", "rclpy_exec_depend.package.xml"},     // <exec_depend> added
    {CPP, CPP_NODE, "rclcpp_empty_depend.package.xml", "rclcpp_depend.package.xml"},     // <depend> added
    {CPP, CPP_NODE, "rclcpp_build_depend.package.xml", "rclcpp_build_and_exec_depend.package.xml"},     // <exec_depend> added
    {CPP, CPP_NODE, "rclcpp_exec_depend.package.xml", "rclcpp_build_and_exec_depend.package.xml"},     // <build_depend> added
    {CPP, CPP_NODE, "rclcpp_build_and_exec_depend.package.xml",
      "rclcpp_build_and_exec_depend.package.xml"},                                                               // Expect no changes
    {CPP, CPP_NODE, "rclcpp_depend.package.xml", "rclcpp_depend.package.xml"},     // Expect no changes
  };

  for (const auto & test_case : test_cases) {
    PackageInfo pkg_info("test_package", get_tmp_package_dest(), test_case.build_type);
    create_package(pkg_info);

    QString xml_path = QDir(pkg_info.package_path).filePath("package.xml");
    QString fixtures_folder = QString(QDir(FIXTURES_PATH).filePath("package_xml_files"));
    QString new_xml_file = QString(QDir(fixtures_folder).filePath(test_case.package_xml_file));
    QString new_content = read_file(new_xml_file);

    write_file(xml_path, new_content, true);
    add_node({"node_1", test_case.node_type}, pkg_info);
    colcon_build(pkg_info.package_destination, {pkg_info.package_name});

    QString current_content = read_file(xml_path);
    QString expected_xml_path =
      QString(QDir(fixtures_folder).filePath(test_case.expected_result_xml_file));
    QString expected_content = read_file(expected_xml_path);
    ASSERT_EQ(current_content, expected_content);
    execute_node_and_assert_success(pkg_info, "node_1", test_case.node_type);
  }
}


// Test listing executables for different package types
TEST(modify_existing_pkg, list_executables) {
  QString src_path = get_tmp_package_dest();
  QString workspace_path = get_workspace_path_without_src(src_path);

  // Create CPP Package
  PackageInfo cpp_package_info("cpp_package", src_path, CPP);
  create_package(cpp_package_info);
  add_node({"cpp_node", NodeType::CPP_NODE}, cpp_package_info);

  // Python Package
  PackageInfo python_package_info("python_package", src_path, PYTHON);
  create_package(python_package_info);
  add_node({"python_node", NodeType::PYTHON_NODE}, python_package_info);

  // Python & CPP
  PackageInfo pkg_info("mixed_package", src_path, CPP_AND_PYTHON);
  create_package(pkg_info);
  add_node({"cpp_node", NodeType::CPP_NODE}, pkg_info);
  add_node({"python_node", NodeType::PYTHON_NODE}, pkg_info);

  // Build all the packages at once to optimize time
  colcon_build(workspace_path);

  QStringList cpp_executables = list_executables(workspace_path, "cpp_package");
  QStringList python_executables = list_executables(workspace_path, "python_package");
  QStringList mixed_executables = list_executables(workspace_path, "mixed_package");

  ASSERT_TRUE(cpp_executables == QStringList("cpp_node"));
  ASSERT_TRUE(python_executables == QStringList("python_node"));
  ASSERT_TRUE(mixed_executables == QStringList({"cpp_node", "python_node"}));
}

// Test regular file (launch/param) listing
TEST(modify_existing_pkg, list_files) {
  PackageInfo pkg_info("cpp_package", get_tmp_package_dest(), CPP);
  create_package(pkg_info);
  create_launch_and_params(
    pkg_info,
    "launch_file_launch",
    "",
    "",
    false
  );

  QStringList launch_files = list_files(QDir(pkg_info.package_path).filePath("launch"));
  ASSERT_TRUE(launch_files == QStringList("launch_file_launch.py"));
}


// Contains a set of larger tests for existing packages that were used for
// sanity-checking that everything works correctly. Currently, there isn't
// a way to run these tests without having locally the expected packages.
TEST(modify_existing_pkg, DISABLED_manual_integration_test_add_new_node) {

  // These packages do not support adding a new node. Skip them.
  QStringList ignored_packages = {"nav2_common", "karto_sdk", "rosbridge_suite",
  "teleop_twist_keyboard"};
  // nav2_common: No ament_package() in CmakeLists.txt
  // karto_sdk: No ament_package() in CmakeLists.txt
  // rosbridge_suite: Meta package
  // teleop_twist_keyboard: declared as py-module in setup.py

  QString workspace_path = "/home/user/ros2_ws/src/integration_test_packages";
  auto packages = list_packages(workspace_path);

  for (auto & package : packages) {
    auto pkg_info = package.second;
    if (ignored_packages.contains(pkg_info.package_name)) {
      continue;
    }

    switch(pkg_info.package_type) {
      case CPP:
        add_node({"cpp_node", CPP_NODE}, pkg_info);
        break;
      case PYTHON:
        add_node({"python_node", PYTHON_NODE}, pkg_info);
        break;
      case CPP_AND_PYTHON:
        add_node({"node_1", CPP_NODE}, pkg_info);
        add_node({"node_2", PYTHON_NODE}, pkg_info);
        break;
      default:
        break;
    }
  }

  colcon_build("/home/user/ros2_ws");

  for (auto & package : packages) {
    auto pkg_info = package.second;
    if (ignored_packages.contains(pkg_info.package_name)) {
      continue;
    }

    QStringList node_names;
    QStringList print_strings;
    switch(pkg_info.package_type) {
      case CPP:
        node_names << "cpp_node";
        print_strings << "C++";
        break;
      case PYTHON:
        node_names << "python_node";
        print_strings << "Python";
        break;
      case CPP_AND_PYTHON:
        node_names << "node_1" << "node_2";
        print_strings << "C++" << "Python";
        break;
      default:
        continue;
    }

    for (int i = 0; i < node_names.size(); ++i) {
      QString node_name = node_names[i];
      QString print_string = print_strings[i];

      QString expected_output = QString("[%1]: Hello world from the %2 node %1").arg(node_name,
        print_string);
      QString output = run_command(
        QString("ros2 run %1 %2").arg(pkg_info.package_name, node_name),
        {expected_output},
        "/home/user/ros2_ws/src"
      );

      ASSERT_TRUE(output.contains(expected_output));
    }
  }
}
