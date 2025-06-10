#ifndef PACKAGEINFO_H
#define PACKAGEINFO_H

#include "turtle_nest/build_type_enum.h"
#include <QDir>
#include <QString>


class PackageInfo
{
public:
    PackageInfo(){};
    QString package_name = "";
    QString package_path = "";
    QString workspace_path = "";
    QString maintainer = "";
    QString description = "";
    QString version = "";
    QString license = "";
    BuildType package_type;

};

#endif // PACKAGEINFO_H
