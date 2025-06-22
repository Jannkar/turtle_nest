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
#include <QDialog>
#include <QButtonGroup>


MainWindow::MainWindow(QWidget * parent, const QString & package_dest)
: QDialog(parent)
  , ui(new Ui::MainWindow), package_destination(package_dest)
{

  ui->setupUi(this);
  setWindowTitle("Create a New Package");

  // Start always from the page 1
  ui->stackedWidget->setCurrentIndex(0);

  // Page 1
  ui->createPackageButton->setVisible(false);
  ui->backButton->setVisible(false);

  if (package_destination.isEmpty()) {
    package_destination = QDir::homePath() + "/ros2_ws/src/";
  }
  ui->workspacePathEdit->setText(package_destination);

  ui->packageNameShortWarn->setVisible(false);
  ui->packageNameEdit->setFocus();

  // Page 2
  // Create a button group for the package type and connect to a signal
  // that connects to a callback that fires if any button is selected
  package_type_group = new QButtonGroup(this);
  package_type_group->addButton(ui->typeCPPButton, 0);
  package_type_group->addButton(ui->typePythonButton, 1);
  package_type_group->addButton(ui->typeMixedButton, 2);
  package_type_group->addButton(ui->typeMsgsButton, 3);
  connect(package_type_group, &QButtonGroup::idToggled,
          this, &MainWindow::handle_package_type_changed);

  ui->launchSuffixWarnLabel->setVisible(false);
  update_package_type_page_ui(get_selected_package_type());

  // Page 3
  ui->invalidEmailLabel->setVisible(false);
  QSettings settings("TurtleNest", "TurtleNest");
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

  if (workspace_path.isEmpty()) {
    return;
  }
  ui->workspacePathEdit->setText(workspace_path);
}


void MainWindow::on_createPackageButton_clicked()
{
  BuildType build_type = get_selected_package_type();

  RosPkgCreator pkg_creator(
    ui->workspacePathEdit->text(),
    ui->packageNameEdit->text(),
    build_type
  );

  QSettings settings("TurtleNest", "TurtleNest");
  settings.setValue("maintainer_name", ui->maintainerEdit->text());
  settings.setValue("maintainer_email", ui->emailEdit->text());

  pkg_creator.description = ui->descriptionEdit->toPlainText();
  pkg_creator.maintainer_name = ui->maintainerEdit->text();

  pkg_creator.node_name_cpp = ui->lineEditNodeNameCpp->text();
  pkg_creator.node_name_python = ui->lineEditNodeNamePython->text();

  if (ui->checkboxCreateLaunch->isChecked()) {
    pkg_creator.launch_name = ui->lineEditLaunchName->text();
  }

  if (ui->checkboxCreateParams->isChecked()) {
    pkg_creator.params_file_name = ui->lineEditParamsName->text();
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
  accept();  // Close and set result as Accepted
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

void MainWindow::handle_package_type_changed(int /*id*/, bool checked){
  if (!checked){
    return;
  }
  BuildType package_type = get_selected_package_type();
  update_package_type_page_ui(package_type);
}

BuildType MainWindow::get_selected_package_type(){
  switch(package_type_group->checkedId()){
    case 0:
      return BuildType::CPP;
    case 1:
      return BuildType::PYTHON;
    case 2:
      return BuildType::CPP_AND_PYTHON;
    case 3:
      return BuildType::MSGS;
    default:
      throw std::runtime_error("Unknown package type");
  }
}

void MainWindow::update_package_type_page_ui(BuildType package_type){
  ui->cppNodewidget->setVisible(false);
  ui->pythonNodeWidget->setVisible(false);
  ui->paramLaunchWidget->setVisible(true);
  ui->lineEditNodeNameCpp->clear();
  ui->lineEditNodeNamePython->clear();

  if (package_type == BuildType::CPP){
    ui->cppNodewidget->setVisible(true);
  } else if (package_type == BuildType::PYTHON){
    ui->pythonNodeWidget->setVisible(true);
  } else if (package_type == BuildType::CPP_AND_PYTHON){
    ui->cppNodewidget->setVisible(true);
    ui->pythonNodeWidget->setVisible(true);
  } else if (package_type == BuildType::MSGS){
    ui->lineEditLaunchName->clear();
    ui->lineEditParamsName->clear();
    ui->checkboxCreateLaunch->setChecked(false);
    ui->checkboxCreateParams->setChecked(false);
    ui->paramLaunchWidget->setVisible(false);
  } else {
    throw std::runtime_error("Unknown package type");
  }
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


void MainWindow::on_checkboxCreateParams_toggled(bool checked)
{
  if (checked) {
    ui->lineEditParamsName->setText(ui->packageNameEdit->text() + "_params");
    ui->lineEditParamsName->setEnabled(true);
  } else {
    ui->lineEditParamsName->setEnabled(false);
    ui->lineEditParamsName->clear();
  }
}


void MainWindow::on_lineEditParamsName_textEdited(const QString & arg1)
{
  autocorrect_line_edit(arg1, ui->lineEditParamsName);
}


void MainWindow::on_lineEditParamsName_editingFinished()
{
  // Use package name as the default params file name if the user tries to leave the field empty
  if (ui->lineEditParamsName->text().length() < 1) {
    ui->lineEditParamsName->setText(ui->packageNameEdit->text() + "_params");
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


void MainWindow::on_paramsNameInfoButton_clicked()
{
  show_tooltip(ui->paramsNameInfoButton);
}

QString MainWindow::get_created_package_name()
{
  return ui->packageNameEdit->text();
}

void show_tooltip(QToolButton * button)
{
  QToolTip::showText(button->mapToGlobal(QPoint(16, 16)), button->toolTip());
}
