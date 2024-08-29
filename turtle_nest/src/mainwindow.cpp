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

#include "turtle_nest/mainwindow.h"
#include "ui_mainwindow.h"
#include "turtle_nest/rospkgcreator.h"
#include "turtle_nest/string_tools.h"
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QProcess>
#include <QLineEdit>
#include <QToolTip>
#include <QDebug>


MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent)
  , ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ui->backButton->setObjectName("low_attention_button");
  ui->browseButton->setObjectName("low_attention_button");

  // Page 1
  ui->createPackageButton->setVisible(false);
  ui->backButton->setVisible(false);
  QSettings settings("TurtleNest", "TurtleNest");
  auto default_workspace_path = QDir::homePath() + "/ros2_ws/src/";
  ui->workspacePathEdit->setText(settings.value("workspace", default_workspace_path).toString());
  ui->packageNameShortWarn->setVisible(false);
  ui->packageNameEdit->setFocus();

  // Page 2
  ui->pythonNodeNameLabel->setVisible(false);
  ui->lineEditNodeNamePython->setVisible(false);
  ui->pythonNodeNameInfoButton->setVisible(false);
  ui->launchSuffixWarnLabel->setVisible(false);

  // Page 3
  ui->invalidEmailLabel->setVisible(false);
  ui->maintainerEdit->setText(settings.value("maintainer_name", "").toString());
  ui->emailEdit->setText(settings.value("maintainer_email", "").toString());
}


MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::on_nextButton_clicked()
{
  ui->backButton->setVisible(true);
  auto last_index = ui->stackedWidget->count() - 1;
  auto new_index = ui->stackedWidget->currentIndex() + 1;
  ui->stackedWidget->setCurrentIndex(new_index);
  if (new_index == last_index) {
    ui->nextButton->setVisible(false);
    ui->createPackageButton->setVisible(true);
  } else {
    ui->nextButton->setVisible(true);
  }
}


void MainWindow::on_backButton_clicked()
{
  ui->nextButton->setVisible(true);
  ui->createPackageButton->setVisible(false);
  auto new_index = ui->stackedWidget->currentIndex() - 1;
  ui->stackedWidget->setCurrentIndex(new_index);
  if (new_index == 0) {
    ui->backButton->setVisible(false);
  }
}


void MainWindow::on_browseButton_clicked()
{
  auto workspace_path = QFileDialog::getExistingDirectory(this, "Select Folder", "");

  // If a folder was selected, save it to the variable
  if (workspace_path.isEmpty()) {
    return;
  }
  ui->workspacePathEdit->setText(workspace_path);
  qInfo() << "Workspace set: " << workspace_path;
}


void MainWindow::on_createPackageButton_clicked()
{
  BuildType build_type;

  if (ui->checkboxCpp->isChecked() && ui->checkboxPython->isChecked()) {
    build_type = CPP_AND_PYTHON;
  } else if (ui->checkboxPython->isChecked()) {
    build_type = PYTHON;
  } else {
    build_type = CPP;
  }

  RosPkgCreator pkg_creator(
    ui->workspacePathEdit->text(),
    ui->packageNameEdit->text(),
    build_type
  );

  QSettings settings("TurtleNest", "TurtleNest");
  settings.setValue("workspace", ui->workspacePathEdit->text());
  settings.setValue("maintainer_name", ui->maintainerEdit->text());
  settings.setValue("maintainer_email", ui->emailEdit->text());

  pkg_creator.description = ui->descriptionEdit->toPlainText();
  pkg_creator.maintainer_name = ui->maintainerEdit->text();

  pkg_creator.node_name_cpp = ui->lineEditNodeNameCpp->text();
  pkg_creator.node_name_python = ui->lineEditNodeNamePython->text();

  if (ui->checkboxCreateLaunch->isChecked()) {
    pkg_creator.launch_name = ui->lineEditLaunchName->text();
  }

  // Row 0 is "No Licence", so it is not directly usable as the license.
  if (ui->licenseList->currentRow() != 0) {
    pkg_creator.license = get_license();
  }

  if (!ui->emailEdit->text().trimmed().isEmpty()) {
    pkg_creator.maintainer_email = ui->emailEdit->text();
  }

  try {
    pkg_creator.create_package();
  } catch (const std::runtime_error & error) {
    qCritical().noquote() << "Package Creation Failed: " << error.what();
    QMessageBox::critical(this, "Package Creation Failed", error.what());
    return;
  }

  qInfo() << "Package created successfully!";
  QString success_msg = "ROS 2 package '" + pkg_creator.package_name +
    "' has been successfully created. You can now build the package.";

  QMessageBox::information(this, "Package Creation Successful", success_msg);

  QApplication::quit();
}


void MainWindow::on_packageNameEdit_textEdited(const QString & arg1)
{
  // When user is editing, short name warning should never be visible.
  ui->packageNameShortWarn->setVisible(false);

  ui->nextButton->setEnabled(true);
  autocorrect_line_edit(arg1, ui->packageNameEdit);
}


void MainWindow::on_packageNameEdit_editingFinished()
{
  if (ui->packageNameEdit->text().length() < 2) {
    ui->packageNameShortWarn->setVisible(true);
    ui->nextButton->setEnabled(false);
  }
}


void MainWindow::change_package_type()
{
  bool cpp_checked = ui->checkboxCpp->isChecked();
  bool python_checked = ui->checkboxPython->isChecked();

  if (cpp_checked && !python_checked) {
    // Only CPP checked
    ui->checkboxCpp->setEnabled(false);
    ui->pythonNodeNameLabel->setVisible(false);
    ui->lineEditNodeNamePython->setVisible(false);
    ui->lineEditNodeNamePython->clear();
    ui->pythonNodeNameInfoButton->setVisible(false);
    ui->cppNodeNameInfoButton->setVisible(true);
  } else if (!cpp_checked && python_checked) {
    // Only Python checked
    ui->checkboxPython->setEnabled(false);
    ui->cppNodeNameLabel->setVisible(false);
    ui->lineEditNodeNameCpp->setVisible(false);
    ui->lineEditNodeNameCpp->clear();
    ui->pythonNodeNameInfoButton->setVisible(true);
    ui->cppNodeNameInfoButton->setVisible(false);
  } else {
    // Both checked
    ui->checkboxCpp->setEnabled(true);
    ui->checkboxPython->setEnabled(true);
    ui->cppNodeNameLabel->setVisible(true);
    ui->lineEditNodeNameCpp->setVisible(true);
    ui->pythonNodeNameLabel->setVisible(true);
    ui->lineEditNodeNamePython->setVisible(true);
    ui->pythonNodeNameInfoButton->setVisible(true);
    ui->cppNodeNameInfoButton->setVisible(true);
  }
}


void MainWindow::on_checkboxCpp_clicked()
{
  change_package_type();
}


void MainWindow::on_checkboxPython_clicked()
{
  change_package_type();
}


void MainWindow::on_lineEditNodeNameCpp_textEdited(const QString & arg1)
{
  autocorrect_line_edit(arg1, ui->lineEditNodeNameCpp);

}


void MainWindow::on_lineEditNodeNamePython_textEdited(const QString & arg1)
{
  autocorrect_line_edit(arg1, ui->lineEditNodeNamePython);
}


void MainWindow::on_checkboxCreateLaunch_toggled(bool checked)
{
  if (checked) {
    ui->lineEditLaunchName->setText(ui->packageNameEdit->text() + "_launch");
    ui->lineEditLaunchName->setEnabled(true);
  } else {
    ui->lineEditLaunchName->setEnabled(false);
    ui->lineEditLaunchName->clear();
    ui->launchSuffixWarnLabel->setVisible(false);
  }
}

void MainWindow::on_lineEditLaunchName_textEdited(const QString & arg1)
{

  QString autocorrected_text = autocorrect_line_edit(arg1, ui->lineEditLaunchName);

  if (autocorrected_text.endsWith("_launch")) {
    ui->launchSuffixWarnLabel->setVisible(false);
  } else {
    ui->launchSuffixWarnLabel->setVisible(true);
  }
}


void MainWindow::on_lineEditLaunchName_editingFinished()
{
  // Use package name as the default launch file name if the user tries to leave the field empty
  if (ui->lineEditLaunchName->text().length() < 1) {
    ui->lineEditLaunchName->setText(ui->packageNameEdit->text() + "_launch");
  }

}

QString MainWindow::get_license()
{
  QListWidgetItem * item = ui->licenseList->currentItem();
  QString license = item->text();
  if (ui->licenseList->currentRow() == 1) {
    license = "Apache-2.0";     // Removes the "(recommended)" suffix
  }
  return license;
}


void MainWindow::on_emailEdit_textEdited(const QString & arg1)
{
  if (is_valid_email(arg1)) {
    ui->createPackageButton->setEnabled(true);
    ui->invalidEmailLabel->setVisible(false);
  }
}


void MainWindow::on_emailEdit_editingFinished()
{
  if (is_valid_email(ui->emailEdit->text())) {
    ui->createPackageButton->setEnabled(true);
    ui->invalidEmailLabel->setVisible(false);
  } else {
    ui->createPackageButton->setEnabled(false);
    ui->invalidEmailLabel->setVisible(true);
  }
}

void MainWindow::on_pkgNameInfoButton_clicked()
{
  show_tooltip(ui->pkgNameInfoButton);
}


void MainWindow::on_cppNodeNameInfoButton_clicked()
{
  show_tooltip(ui->cppNodeNameInfoButton);
}


void MainWindow::on_pythonNodeNameInfoButton_clicked()
{
  show_tooltip(ui->pythonNodeNameInfoButton);
}

void MainWindow::on_launchNameInfoButton_clicked()
{
  show_tooltip(ui->launchNameInfoButton);
}

void show_tooltip(QToolButton * button)
{
  QToolTip::showText(button->mapToGlobal(QPoint(16, 16)), button->toolTip());
}
