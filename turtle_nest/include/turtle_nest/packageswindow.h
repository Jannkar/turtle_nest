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

class PackagesWindow: public QMainWindow
{
  Q_OBJECT

public:
  explicit PackagesWindow(QWidget * parent = nullptr);
  ~PackagesWindow();

private:
  const QString default_workspace_path = QDir::homePath() + "/ros2_ws/";
  QSettings settings;

private slots:
  void on_packagesListWidget_currentItemChanged(
    QListWidgetItem * current,
    QListWidgetItem * previous);
  PackageInfo & get_selected_package_info();

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
  Ui::PackagesWindow * ui;
  PackageInfoForm * package_info_form;
  std::map < QString, PackageInfo > packages;
  QString workspace_path;
  bool first_package_refresh = true;
};

#endif // PACKAGESWINDOW_H
