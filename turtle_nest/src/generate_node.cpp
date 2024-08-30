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

#include "turtle_nest/generate_node.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/string_tools.h"

#include <QDir>
#include <QDebug>

void generate_python_node(QString workspace_path, QString package_name, QString node_name)
{
  QString package_path = QDir(workspace_path).filePath(package_name);
  QString node_dir = QDir(package_path).filePath(package_name);
  QString node_path = QDir(node_dir).filePath(node_name + ".py");

  QString content = QString(
    R"(#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class %2(Node):

    def __init__(self):
        super().__init__("%1")
        self.get_logger().info("Hello world from the Python node %1")


def main(args=None):
    rclpy.init(args=args)

    %1 = %2()

    try:
        rclpy.spin(%1)
    except KeyboardInterrupt:
        pass

    %1.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
)")
    .arg(node_name, to_camel_case(node_name));

  create_directory(node_dir);
  write_file(node_path, content);
  qInfo() << "Generated new Python node " + node_name;
  add_exec_permissions(node_path);
}

void create_init_file(QString node_dir)
{
  create_directory(node_dir);
  QString init_path = QDir(node_dir).filePath("__init__.py");
  write_file(init_path, "");
}

void add_exec_permissions(QString node_path)
{
  QFile file(node_path);
  QFileDevice::Permissions current_permissions = file.permissions();

  QFileDevice::Permissions new_permissions = current_permissions |
    QFileDevice::ExeOwner |
    QFileDevice::ExeGroup |
    QFileDevice::ExeOther;

  if (!file.setPermissions(new_permissions)) {
    QString error_msg = QString("Failed to set execution permissions for Python file '%1'").arg(
      node_path);
    qCritical() << error_msg;
    throw std::runtime_error(error_msg.toStdString());
  }
}

void add_py_node_to_cmake(QString c_make_file_path, QString package_name, QString node_name)
{
  QString content("# Install Python modules\n"
    "ament_python_install_package(${PROJECT_NAME})\n\n"
    "# Install Python executables\n"
    "install(PROGRAMS\n"
    "    %1/%2.py\n"
    "DESTINATION lib/${PROJECT_NAME}\n"
    "RENAME %2"
    ")\n\n");
  content = content.arg(package_name, node_name);
  append_to_file_before(c_make_file_path, content, "ament_package()");
}


void generate_cpp_node(QString package_path, QString node_name)
{
  // Uncrustify considers this as a single line. Skip.
  /* *INDENT-OFF* */
    QString content = QString(R"(#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class %2 : public rclcpp::Node
{
  public:
    %2()
    : Node("%1")
    {
      RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "%1");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<%2>());
  rclcpp::shutdown();
  return 0;
})").arg(node_name, to_camel_case(node_name));
  /* *INDENT-ON* */
  write_file(QDir(package_path).filePath("src/" + node_name + ".cpp"), content);
}
