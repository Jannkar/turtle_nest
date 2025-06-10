#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <QString>
#include "turtle_nest/build_type_enum.h"
#include <turtle_nest/packageinfo.h>
#include <turtle_nest/rospkgcreator.h>

bool file_exists(QString path);
bool string_exists_in_file(const QString& filePath, const QString& searchString);
QString get_tmp_workspace_path();
bool dir_is_empty(const QString& dirPath);

// Expects the workspace_path with /src ending
QString run_command(QString command, QStringList outputs_to_wait, QString workspace_path = "");

std::tuple<PackageInfo, RosPkgCreator> create_temp_package(BuildType build_type, QString pkg_name="test_package");
#endif // TEST_UTILS_H
