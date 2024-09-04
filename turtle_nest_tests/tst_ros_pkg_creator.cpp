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

QString run_command(QString command, QStringList outputs_to_wait, QString workspace_path = ""){
    QDateTime current_date_time = QDateTime::currentDateTime();
    qint64 timestamp_seconds = current_date_time.toSecsSinceEpoch();
    qint64 timestamp_ms = current_date_time.toMSecsSinceEpoch();
    QString formatted_time = QString("[%1.%2]")
                                  .arg(timestamp_seconds)
                                  .arg(timestamp_ms % 1000, 3, 10, QChar('0'));

    QProcess *process = new QProcess();
    process->setProcessChannelMode(QProcess::MergedChannels);

    // Source the workspace if given
    if (!workspace_path.isEmpty()) {
        QDir dir(workspace_path);
        dir.cdUp(); // Get the path without /src extension
        command = QString("source %1/install/setup.bash && %2").arg(dir.path(), command);
    }

    qDebug().noquote() << formatted_time << "Running:" << "bash -c" << command;
    process->start("bash", QStringList() << "-c" << command);

    // Wait for the command to execute until all "outputs_to_wait" strings are found from the output
    QElapsedTimer timer;
    timer.start();

    QString output;
    while (!outputs_to_wait.isEmpty() && timer.elapsed() < 30000) {
        if (!process->waitForReadyRead(100)) {
            continue;
        }
        QString new_output = process->readAllStandardOutput();
        output += new_output;
        qDebug().noquote() << new_output;

        // Check if any expected output is found in the output
        for (int i = 0; i < outputs_to_wait.size(); ) {
            if (output.contains(outputs_to_wait[i])) {
                outputs_to_wait.removeAt(i);
            } else {
                ++i;
            }
        }
    }

    if (outputs_to_wait.isEmpty()){
        qDebug() << "Found all the expected lines";
    } else {
        qDebug() << "Timed out while waiting for the expected lines";
    }

    // Sending SIGINT very soon after the launch was started will make the process hang. Wait for a second before doing that.
    qDebug() << "Sending SIGINT";
    QThread::sleep(1);
    kill(process->processId(), SIGINT);

    if (!process->waitForFinished(20000)) {
        // If it still didn't finish, terminate
        qCritical() << "Process didn't finish with SIGINT. Terminating. This might leave active background processes";
        process->terminate();
    }

    QString new_output = process->readAllStandardOutput();
    output += new_output;
    qDebug().noquote() << new_output;

    return output;
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

    QString output = run_command("ros2 launch package_name demo_launch.py", {python_node_text, cpp_node_text}, pkg_creator.workspace_path);
    ASSERT_TRUE(output.contains(python_node_text));
    ASSERT_TRUE(output.contains(cpp_node_text));
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
    pkg_creator.launch_name = "test_launch";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));

    QString output = run_command("ros2 launch package_name test_launch.py", {cpp_node_text}, pkg_creator.workspace_path);
    ASSERT_TRUE(output.contains(cpp_node_text));
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
    pkg_creator.launch_name = "test_launch";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/python_node.py"));
    ASSERT_TRUE(string_exists_in_file(
        (pkg_creator.package_path + "/package_name/python_node.py"),
        "Hello world from the Python node python_node"
    ));

    QString output = run_command("ros2 launch package_name test_launch.py", {python_node_text}, pkg_creator.workspace_path);
    ASSERT_TRUE(output.contains(python_node_text));
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
    pkg_creator.launch_name = "test_launch";
    pkg_creator.create_package();
    pkg_creator.build_package();
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/src/cpp_node.cpp"));
    ASSERT_TRUE(file_exists(pkg_creator.package_path + "/package_name/__init__.py"));

    QString output = run_command("ros2 launch package_name test_launch.py", {cpp_node_text}, pkg_creator.workspace_path);
    ASSERT_TRUE(output.contains(cpp_node_text));
}

// Create a CPP+Python package with Python Node
TEST(ros_pkg_creator, cpp_python_with_python_node){
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

    QString output = run_command("ros2 launch package_name test_launch.py", {python_node_text}, pkg_creator.workspace_path);
    ASSERT_TRUE(output.contains(python_node_text));
}


// Create a CPP+Python package with both Nodes
TEST(ros_pkg_creator, cpp_python_with_both_nodes){
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
    QString output = run_command(QString("ros2 launch package_name test_launch.py"), outputs_to_wait, pkg_creator.workspace_path);
    for (const QString &expected : outputs_to_wait) {
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

    QString output = run_command(QString("ros2 launch test_package test_launch.py"), {"[INFO] [launch]:"}, pkg_creator.workspace_path);

    ASSERT_TRUE(!output.contains(python_param_text));
    ASSERT_TRUE(!output.contains(cpp_param_text));

    QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
    ASSERT_TRUE(file_exists(params_path));
    ASSERT_EQ(read_file(params_path), "");

    ASSERT_TRUE(string_exists_in_file((pkg_creator.package_path + "/CMakeLists.txt"), "# Install config files"));
}

// Test that an empty params file is created when package with no Nodes is created (Python package)
TEST(ros_pkg_creator, test_params_with_no_nodes_python) {
    RosPkgCreator pkg_creator(get_tmp_workspace_path(), "test_package", PYTHON);
    pkg_creator.launch_name = "test_launch";
    pkg_creator.params_file_name = "test_params";
    pkg_creator.create_package();
    pkg_creator.build_package();

    QString output = run_command(QString("ros2 launch test_package test_launch.py"), {"[INFO] [launch]:"}, pkg_creator.workspace_path);

    ASSERT_TRUE(!output.contains(python_param_text));
    ASSERT_TRUE(!output.contains(cpp_param_text));

    QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
    ASSERT_TRUE(file_exists(params_path));
    ASSERT_EQ(read_file(params_path), "");

    ASSERT_TRUE(string_exists_in_file((pkg_creator.package_path + "/setup.py"), "(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),"));
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
    QString output = run_command(QString("ros2 launch test_package test_launch.py"), outputs_to_wait, pkg_creator.workspace_path);

    for (const QString &expected : outputs_to_wait) {
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

    QStringList outputs_to_wait = {python_node_text, cpp_node_text, python_param_text, cpp_param_text};
    QString output = run_command(QString("ros2 launch test_package test_launch.py"), outputs_to_wait, pkg_creator.workspace_path);

    for (const QString &expected : outputs_to_wait) {
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

    QString output = run_command(QString("ros2 launch test_package test_launch.py"), outputs_to_wait, pkg_creator.workspace_path);

    for (const QString &expected : outputs_to_wait) {
        ASSERT_TRUE(output.contains(expected));
    }

    ASSERT_TRUE(!output.contains(python_param_text));
    ASSERT_TRUE(!output.contains(cpp_param_text));

    QString params_path = QString(pkg_creator.package_path + "/config/test_params.yaml");
    ASSERT_TRUE(!file_exists(params_path));

    // Confirm that parameters are empty in the launch file
    ASSERT_TRUE(string_exists_in_file((pkg_creator.workspace_path + "/test_package" + "/launch/" + pkg_creator.launch_name + ".py"), "parameters=[]"));

}
