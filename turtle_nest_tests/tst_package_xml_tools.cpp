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
#include "turtle_nest/file_utils.h"
#include "turtle_nest/package_xml_tools.h"
#include <QTemporaryDir>
#include <QDebug>


class PackageXMLTest : public ::testing::Test {
protected:
  QTemporaryDir temp_dir;

  QString copy_fixture(QString fixture_name)
  {
    QString fixture_path = QDir(FIXTURES_PATH).filePath(fixture_name);
    QString temp_file = temp_dir.filePath("package.xml");
    if (!QFile::copy(fixture_path, temp_file)) {
      throw std::runtime_error("Failed to copy file");
    }
    return temp_dir.path();     // Return the folder
  }
};


TEST_F(PackageXMLTest, happy_flow) {
  QString package_dir = copy_fixture("package_xml_files/rclpy_depend.package.xml");
  PackageXMLEditor editor(package_dir);
  ASSERT_TRUE(editor.has_dependency("rclpy", DependencyType::DEPEND));
}

TEST_F(PackageXMLTest, try_open_empty_xml) {
  QString package_dir = copy_fixture("package_xml_files/general_empty_file.package.xml");
  ASSERT_THROW({
    PackageXMLEditor editor(package_dir);
  }, std::runtime_error);
}

TEST_F(PackageXMLTest, try_open_non_existing_xml) {
  QTemporaryDir temp_dir;
  QString temp_file_path = temp_dir.filePath("empty.xml");
  ASSERT_THROW({
    PackageXMLEditor editor(temp_file_path);
  }, std::runtime_error);
}

// Rest of the functions are tested as part of integration tests
