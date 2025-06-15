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

#include "turtle_nest/packageswindow.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/modify_existing_pkg.h"
#include "ui_packageswindow.h"

#include <QScrollBar>
#include <QFileDialog>
#include <QSplitter>
#include <QDebug>
#include <QMessageBox>
#include <QMovie>
#include <QtConcurrent>

#include <turtle_nest/mainwindow.h>
#include <turtle_nest/packageinfoform.h>

PackagesWindow::PackagesWindow(QWidget * parent)
: QMainWindow(parent)
  , settings("TurtleNest", "TurtleNest")
  , ui(new Ui::PackagesWindow)
{
  ui->setupUi(this);

  //Setup qss
  ui->welcomePage->setObjectName("welcomePage");
  ui->emptyPackagesPage->setObjectName("welcomePage");
  ui->workspacePathEdit->setObjectName("mainWorkspacePathEdit");

  // Populate PackageInfoForm
  package_info_form = new PackageInfoForm(ui->scrollAreaWidgetContents);
  ui->scrollAreaLayout->addWidget(package_info_form);

  // Set buttons invisible
  ui->editOkButton->setVisible(false);
  ui->browseButton->setVisible(false);
  ui->cancelButton->setVisible(false);

  // Set the splitter options
  ui->splitter->setStretchFactor(0, 0);    // Side panel smaller
  ui->splitter->setStretchFactor(1, 2);    // Main view takes more space
  ui->splitter->setHandleWidth(8);
  ui->splitter->setCollapsible(0, false);    // Prevents first widget (sidebar) from collapsing
  ui->splitter->setCollapsible(1, false);    // Prevents second widget (package info form) from collapsing

  // When workspace is edited and ENTER pressed, presses the OK button.
  connect(ui->workspacePathEdit, &QLineEdit::returnPressed, ui->editOkButton, &QPushButton::click);

  // Setup the workspace
  QString workspace_path = settings.value("package_workspace", "").toString();
  set_workspace(workspace_path);
}

PackagesWindow::~PackagesWindow()
{
  delete ui;
}


void PackagesWindow::refresh_package_list(QString select_package)
{
  // Show the loading spinner and disable UI clicks
  show_loading_spinner();
  ui->centralwidget->setEnabled(false);

  // Run the package listing in a separate thread, as it might take a while.
  (void)QtConcurrent::run(
    [this, select_package]() {
      packages = list_packages(workspace_path);

      QMetaObject::invokeMethod(
        this, [this, select_package]() {
          // Signal blocks required, otherwise slots like "current_item_changed"
          // will fire and cause a crash
          ui->packagesListWidget->blockSignals(true);
          ui->packagesListWidget->clear();
          ui->packagesListWidget->blockSignals(false);

          for (const auto & [pkg_name, pkg_info] : packages) {
            QListWidgetItem * item = new QListWidgetItem(pkg_info.package_name);
            item->setToolTip(pkg_info.package_path);
            ui->packagesListWidget->addItem(item);
          }

          // Reselect the previous item if possible
          QListWidgetItem * selected_item = nullptr;
          if (!select_package.isEmpty()) {
            for (int i = 0; i < ui->packagesListWidget->count(); ++i) {
              QListWidgetItem * item = ui->packagesListWidget->item(i);
              if (item->text() == select_package) {
                selected_item = item;
                break;
              }
            }
          }

          // If the workspace has been set, hide the welcome label.
          if (!workspace_path.isEmpty()) {
            ui->welcomeLabel->hide();
            ui->refreshWorkspaceButton->show();
          } else {
            ui->welcomeLabel->show();
            ui->refreshWorkspaceButton->hide();
          }

          // Figure out which view to show
          bool workspace_selected = !workspace_path.isEmpty();
          bool has_packages = !packages.empty();

          int page_index = 1;    // default: “Turtle Nest Logo” view

          if (workspace_selected) {
            if (!has_packages) {
              page_index = 2;        // “No ROS 2 packages” view
            } else if (!first_package_refresh) {
              page_index = 0;        // regular package-info view
            }
          }

          ui->stackedPackageInfoWidget->setCurrentIndex(page_index);

          // If we’re on the package list page, set the selection
          if (page_index == 0) {
            if (selected_item) {
              ui->packagesListWidget->setCurrentItem(selected_item);
            } else {
              ui->packagesListWidget->setCurrentRow(0);
            }
          }

          first_package_refresh = false;
          hide_loading_spinner();
          ui->centralwidget->setEnabled(true);
        }, Qt::QueuedConnection);
    });
}

void PackagesWindow::on_packagesListWidget_currentItemChanged(QListWidgetItem *, QListWidgetItem *)
{
  // Make sure the package info-form is shown.
  ui->stackedPackageInfoWidget->setCurrentIndex(0);

  package_info_form->updatePackageInfo(get_selected_package_info());
  ui->scrollArea->verticalScrollBar()->setValue(0);
}

PackageInfo & PackagesWindow::get_selected_package_info()
{
  return packages[ui->packagesListWidget->currentItem()->text().toLower()];
}


void PackagesWindow::on_browseButton_clicked()
{
  auto new_workspace_path = QFileDialog::getExistingDirectory(this, "Select Folder", "");

  if (new_workspace_path.isEmpty()) {
    return;
  }

  ui->workspacePathEdit->setText(new_workspace_path);
}

void PackagesWindow::on_createNewPackageButton_clicked()
{
  MainWindow mainWindow(this, get_workspace_path_with_src(workspace_path));
  if (mainWindow.exec() == QDialog::Accepted) {    // Check if accepted
    QString pkg_name = mainWindow.get_created_package_name();
    refresh_package_list(pkg_name);
  }
}


void PackagesWindow::on_editWorkspaceButton_clicked()
{
  // For some reason, the following line:
  // ui->editWorkspaceButton->setVisible(!modify_path_mode_on);
  // in the set_modify_path_mode causes packages list
  // to trigger on_changed. Block the signals to avoid this.
  ui->packagesListWidget->blockSignals(true);
  set_modify_path_mode(true);
  ui->packagesListWidget->blockSignals(false);
}


void PackagesWindow::on_editOkButton_clicked()
{
  // Make sure that ENTER press doesn't trigger OK when the
  // workspacePathEdit is readOnly
  if (ui->workspacePathEdit->isReadOnly()) {
    return;
  }

  auto new_workspace_path = ui->workspacePathEdit->text();

  // Create a new workspace if it doesn't yet exist, and we don't have an empty path
  if (!new_workspace_path.isEmpty()) {
    QDir dir(new_workspace_path);
    if (!dir.exists()) {
      if (!create_new_workspace(new_workspace_path)) {
        return;
      }
    }
  }

  set_workspace(new_workspace_path);
  set_modify_path_mode(false);
}


void PackagesWindow::set_modify_path_mode(bool modify_path_mode_on)
{
  // Set a default workspace path, if we are going into
  // edit mode and no workspace is yet set
  if (modify_path_mode_on && ui->workspacePathEdit->text().isEmpty()) {
    ui->workspacePathEdit->setText(default_workspace_path);
  }

  ui->workspacePathEdit->setReadOnly(!modify_path_mode_on);
  ui->editOkButton->setVisible(modify_path_mode_on);
  ui->browseButton->setVisible(modify_path_mode_on);
  ui->cancelButton->setVisible(modify_path_mode_on);
  ui->editWorkspaceButton->setVisible(!modify_path_mode_on);

  // Set the refresh button visible only if we have a workspace set.
  if (!modify_path_mode_on && !workspace_path.isEmpty()) {
    ui->refreshWorkspaceButton->setVisible(true);
  } else {
    ui->refreshWorkspaceButton->setVisible(false);
  }

  if (modify_path_mode_on) {
    ui->workspacePathEdit->setFocus();

    // Move cursor to the end
    ui->workspacePathEdit->setCursorPosition(ui->workspacePathEdit->text().length());
  }
}

void PackagesWindow::set_workspace(QString new_workspace)
{
  workspace_path = new_workspace;
  settings.setValue("package_workspace", new_workspace);

  ui->workspacePathEdit->setText(new_workspace);
  qDebug() << "Workspace set: " << new_workspace;
  refresh_package_list();
}

bool PackagesWindow::create_new_workspace(QString new_workspace_path)
{
  QDir dir(new_workspace_path);

  // Ask the user if they want to create the folder
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(
    this, "Create Folder",
    "Create a new workspace?\n\n" + new_workspace_path,
    QMessageBox::No | QMessageBox::Yes);

  if (reply == QMessageBox::Yes) {
    if (!QFileInfo(new_workspace_path).isAbsolute()) {
      QMessageBox::critical(this, "Error", "Failed to create folder! Invalid path.");
      return false;
    }
    if (!dir.mkpath(new_workspace_path)) {
      QMessageBox::critical(
        this, "Error",
        "Failed to create folder! Is the path valid and do you have sufficient permissions?");
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void PackagesWindow::on_cancelButton_clicked()
{
  ui->workspacePathEdit->setText(workspace_path);
  set_modify_path_mode(false);
}

void PackagesWindow::on_refreshWorkspaceButton_clicked()
{
  QListWidgetItem * item = ui->packagesListWidget->currentItem();
  QString current_selection;

  if (item) {
    current_selection = item->text();
  } else {
    current_selection = "";
  }

  refresh_package_list(current_selection);
}

void PackagesWindow::show_loading_spinner()
{
  // Create the loading label (spinner)
  QLabel * loadingLabel = new QLabel(this);
  loadingLabel->setObjectName("loadingLabel");
  QMovie * movie = new QMovie(":/images/img/Spin@1x-1.0s-80px-80px.gif");
  loadingLabel->setMovie(movie);
  movie->start();

  // Set the label to cover the entire window area
  loadingLabel->setAlignment(Qt::AlignCenter);
  loadingLabel->setGeometry(0, 0, width(), height());

  // make sure the label doesn't block clicks
  loadingLabel->setAttribute(Qt::WA_TransparentForMouseEvents);

  // Set the window flags so it floats above other widgets
  loadingLabel->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
  loadingLabel->show();
}

void PackagesWindow::hide_loading_spinner()
{
  // Find the label in the window and hide it
  foreach(QWidget * widget, this->findChildren<QWidget *>()) {
    if (widget->objectName() == "loadingLabel") {
      widget->hide();
      widget->deleteLater();
    }
  }
}
