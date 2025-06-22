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

#include "turtle_nest/generate_msgs_pkg.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/package_xml_tools.h"
#include <QDebug>
#include <QDir>

void add_msgs_to_cmakelists(QString package_path){
  QString c_make_path = QDir(package_path).filePath("CMakeLists.txt");
  QString append_before_text = "ament_package()";
  append_to_file_before(c_make_path, get_msgs_cmake_addition(), append_before_text);
}

void create_msgs_files(QString package_path){
  QString msgs_file_path = "msg/Example.msg";
  QString full_path = QDir(package_path).filePath(msgs_file_path);
  write_file(full_path, get_example_msg_contents());
  PackageXMLEditor xml_editor = PackageXMLEditor(package_path);
  xml_editor.add_dependency("rosidl_default_generators", DependencyType::BUILDTOOL_DEPEND);
  xml_editor.add_dependency("rosidl_default_runtime", DependencyType::EXEC_DEPEND);
  xml_editor.add_dependency("rosidl_interface_packages", DependencyType::MEMBER_OF_GROUP);
}

QString get_msgs_cmake_addition(){
  return QString(
      R"(# Custom ROS 2 messages
find_package(rosidl_default_generators REQUIRED)

# Add here your custom message (.msg), service (.srv), and action (.action) files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Example.msg"
)

)");
}

QString get_example_msg_contents(){
  return QString(
      R"(# This is a custom ROS 2 message definition.
# Each line defines one field, consisting of a type and a name.

string example_data


# Supported built-in data types:
# - bool
# - byte
# - char
# - float32
# - float64
# - int8
# - int16
# - int32
# - int64
# - string
# - uint8
# - uint16
# - uint32
# - uint64
# - wstring
#
# Any built-in type can be also defined as array, for example:
# - string[] (unbounded array)
# - string[5] (bounded, five string elements)
)");
}
