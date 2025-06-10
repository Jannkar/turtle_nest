#ifndef MODIFY_EXISTING_PKG_H
#define MODIFY_EXISTING_PKG_H

#include "turtle_nest/packageinfo.h"
#include <QString>
#include <QDir>

std::map<QString, PackageInfo> list_packages(QString workspace_path);
QStringList list_executables(const QString workspace_path, const QString package_name);
QStringList list_files(const QString path);
bool is_src_package(QDir dir);
bool is_python_package(QString package_path);
bool is_mixed_package(QString package_path, QString package_name);
bool is_cpp_package(QString package_path);
bool is_msgs_package(QString package_name);
BuildType get_package_build_type(QString package_path, QString package_name);
QString clean_description(const QString &input);

#endif // MODIFY_EXISTING_PKG_H
