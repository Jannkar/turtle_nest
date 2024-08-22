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
#include <turtle_nest/rospkgcreator.h>
#include <QDebug>
#include <QTemporaryDir>
#include <QXmlStreamReader>
#include <QRegularExpression>


using namespace testing;


bool file_exists(QString path) {
    QFile file(path);
    return file.exists();
}


bool string_exists_in_file(const QString& filePath, const QString& searchString) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Error opening file:" << file.errorString();
        return false;
    }

    QTextStream in(&file);

    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.contains(searchString, Qt::CaseSensitive)) {
            file.close();
            return true;
        }
    }

    file.close();
    return false;
}

QString read_xml_tag(const QString &filePath, const QString &tagName, const QString &attributeName = QString()) {
    QFile file(filePath);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Error opening file:" << file.errorString();
        return QString();
    }

    QXmlStreamReader xmlReader(&file);
    QString result;

    // Parse the XML file
    while (!xmlReader.atEnd() && !xmlReader.hasError()) {
        QXmlStreamReader::TokenType token = xmlReader.readNext();

        // If token is a StartElement and the tag name matches
        if (token == QXmlStreamReader::StartElement) {
            if (xmlReader.name() == tagName) {
                if (!attributeName.isEmpty() && xmlReader.attributes().hasAttribute(attributeName)) {
                    // If attribute is specified and exists, return the attribute value
                    result = xmlReader.attributes().value(attributeName).toString();
                } else {
                    // Otherwise, return the tag's text content
                    result = xmlReader.readElementText();
                }
                break; // Stop after finding the first matching tag or attribute
            }
        }
    }

    if (xmlReader.hasError()) {
        qDebug() << "Error reading XML file:" << xmlReader.errorString();
    }

    file.close();
    return result;
}

QString get_tmp_workspace_path(){
    QTemporaryDir temp_dir;
    return temp_dir.path() + "/ros2_ws/src";
}

bool dir_is_empty(const QString& dirPath) {
    QDir dir(dirPath);
    if (!dir.exists()) {
        qDebug() << "The directory does not exist.";
        throw std::runtime_error("Directory does not exist");
    }
    // Filter out "." and ".." which are always present in directories
    QStringList files = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);
    qInfo() << files;
    return files.isEmpty();
}


// Test the package creation with default parameters
TEST(ros_pkg_creator, create_pkg_defaults){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP);
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.workspace_path + "/package_name/CMakeLists.txt"));
}

// Test the package creation with all the parameters
TEST(ros_pkg_creator, create_pkg_all_values){
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

}

// Try to create a package to destination the user doesn't have access rights
TEST(ros_pkg_creator, workspace_creation_fails){
    RosPkgCreator pkg_creator("/root/no_permission", "package_name", CPP);
    EXPECT_THROW({
        pkg_creator.create_package();
    }, std::runtime_error);
}

// Create a C++ package without a Node
TEST(ros_pkg_creator, cpp_no_node){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP);
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(dir_is_empty(pkg_creator.package_path + "/src"));
}

// Create a C++ package with a Node
TEST(ros_pkg_creator, cpp_with_node){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP);
    pkg_creator.node_name_cpp = "cpp_node";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));
}

// Create a Python package without a Node
TEST(ros_pkg_creator, python_no_node){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", PYTHON);
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
}


// Create a Python package with a Node
TEST(ros_pkg_creator, python_with_node){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", PYTHON);
    pkg_creator.node_name_python = "python_node";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/python_node.py"));
    ASSERT_TRUE(string_exists_in_file(
        (pkg_creator.package_path + "/package_name/python_node.py"),
        "Hello world from the Python node python_node"
    ));
}

// Create a CPP+Python package without Nodes
TEST(ros_pkg_creator, cpp_python_no_nodes){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(dir_is_empty(pkg_creator.package_path + "/src"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
}

// Create a CPP+Python package with CPP Node
TEST(ros_pkg_creator, cpp_python_with_cpp_node){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
    pkg_creator.node_name_cpp = "cpp_node";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
}

// Create a CPP+Python package with Python Node
TEST(ros_pkg_creator, cpp_python_with_python_node){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
    pkg_creator.node_name_python = "python_node";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(dir_is_empty(pkg_creator.package_path + "/src"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/python_node.py"));
    ASSERT_TRUE(string_exists_in_file(
        (pkg_creator.package_path + "/package_name/python_node.py"),
        "Hello world from the Python node python_node"
        ));
}


// Create a CPP+Python package with both Nodes
TEST(ros_pkg_creator, cpp_python_with_both_nodes){
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "package_name", CPP_AND_PYTHON);
    pkg_creator.node_name_cpp = "cpp_node";
    pkg_creator.node_name_python = "python_node";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/python_node.py"));
    ASSERT_TRUE(string_exists_in_file(
        (pkg_creator.package_path + "/package_name/python_node.py"),
        "Hello world from the Python node python_node"
        ));
}


