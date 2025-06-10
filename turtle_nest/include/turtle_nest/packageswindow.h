#ifndef PACKAGESWINDOW_H
#define PACKAGESWINDOW_H

#include "turtle_nest/packageinfo.h"
#include "turtle_nest/packageinfoform.h"

#include <QDir>
#include <QLabel>
#include <QListWidgetItem>
#include <QMainWindow>
#include <QSettings>


namespace Ui {
class PackagesWindow;
}

class PackagesWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PackagesWindow(QWidget *parent = nullptr);
    ~PackagesWindow();

private:
    const QString default_workspace_path = QDir::homePath() + "/ros2_ws/";
    QSettings settings;

private slots:
    void on_packagesListWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);
    PackageInfo& get_selected_package_info();

    void on_browseButton_clicked();

    void refresh_package_list(QString select_package = "");

    void on_createNewPackageButton_clicked();

    void on_editWorkspaceButton_clicked();

    void on_editOkButton_clicked();

    void set_modify_path_mode(bool modify_path_mode_on);

    void set_workspace(QString new_workspace);

    bool create_new_workspace(QString new_workspace_path);

    void on_cancelButton_clicked();

    void on_refreshWorkspaceButton_clicked();

    void show_loading_spinner();

    void hide_loading_spinner();

private:
    Ui::PackagesWindow *ui;
    PackageInfoForm *package_info_form;
    std::map<QString, PackageInfo> packages;
    QString workspace_path;
    bool first_package_refresh = true;
};

#endif // PACKAGESWINDOW_H
