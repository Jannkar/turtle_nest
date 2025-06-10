#ifndef PACKAGEINFOFORM_H
#define PACKAGEINFOFORM_H

#include "turtle_nest/packageinfo.h"
#include <QGraphicsSvgItem>
#include <QGraphicsView>
#include <QLabel>
#include <QSvgWidget>
#include <QWidget>
#include <QListWidget>

namespace Ui {
class PackageInfoForm;
}

class PackageInfoForm : public QWidget
{
    Q_OBJECT

public:
    explicit PackageInfoForm(QWidget *parent = nullptr);
    ~PackageInfoForm();
    void updatePackageInfo(PackageInfo pkg_info);
    QSvgWidget *createScaledSvg(QString path, int targetHeight);
    void updateNodesLaunchParams(PackageInfo pkg_info);
private slots:
    void on_addNodeButton_clicked();

private:
    Ui::PackageInfoForm *ui;
    QSvgWidget *cpp_svg;
    QSvgWidget *python_svg;
    QSvgWidget *msgs_svg;
    PackageInfo current_package_info;
};

void adjustListWidgetHeight(QListWidget *listWidget);

#endif // PACKAGEINFOFORM_H
