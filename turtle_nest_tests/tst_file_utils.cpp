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
#include <QTemporaryDir>
#include <filesystem>

namespace fs = std::filesystem;


class FileUtilsTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_dir_path = QDir::tempPath() + "/turtle_nest_test_dir/";
    }

    void TearDown() override {
        QDir dir(test_dir_path);
        if (dir.exists()) {
            dir.removeRecursively();
        }
    }

    QString test_dir_path;
};

using namespace testing;


TEST_F(FileUtilsTest, create_directory_happy_flow){
    QDir dir(test_dir_path);
    ASSERT_FALSE(dir.exists());
    create_directory(test_dir_path);
    ASSERT_TRUE(dir.exists());
}

TEST_F(FileUtilsTest, create_dir_already_exists)
{
    // Try to create directory that already exists
    QTemporaryDir temp_dir;
    ASSERT_TRUE(temp_dir.isValid());
    create_directory(temp_dir.path());

    // Check that the directory still exists
    ASSERT_TRUE(fs::is_directory(temp_dir.path().toStdString()));
}


TEST_F(FileUtilsTest, directory_creation_failed){
    EXPECT_THROW({
        create_directory("/restricted_directory/test_dir");
    }, std::runtime_error);
}

