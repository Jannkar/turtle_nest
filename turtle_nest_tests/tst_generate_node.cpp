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
#include "turtle_nest/generate_node.h"
#include "turtle_nest/file_utils.h"
#include <QTemporaryDir>

using namespace testing;

// Tests that the python node is correctly generated
TEST(generate_node, happy_flow)
{
    QTemporaryDir temp_dir;

    // Create mock CMakeLists
    QString package_path = QDir(temp_dir.path()).filePath("package_1");
    QString mock_cmake_path = QDir(package_path).filePath("CMakeLists.txt");
    create_directory(package_path);
    write_file(mock_cmake_path, "ament_package()");

    generate_python_node(temp_dir.path(), "package_1", "node_123", false);

    QString node_path = QDir(temp_dir.path()).filePath("package_1/package_1/node_123.py");
    QString contents = read_file(node_path);
    ASSERT_TRUE(contents.indexOf("Hello world from the Python node node_123") != -1);
}
