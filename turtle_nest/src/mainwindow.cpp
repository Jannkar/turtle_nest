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
#include "turtle_nest/node_generators/node_generator_factory.h"
#include "turtle_nest/package_generators/base_package_generator.h"
#include "turtle_nest/package_generators/create_package.h"
#include "ui_mainwindow.h"
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
#include <turtle_nest/node_generators/base_node_generator.h>


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
  connect(
    package_type_group, &QButtonGroup::idToggled,
    this, &MainWindow::handle_package_type_changed);

  ui->launchSuffixWarnLabel->setVisible(false);
  update_package_type_page_ui(get_selected_package_type());

  // Page 4
  ui->invalidEmailLabel->setVisible(false);
  QSettings settings("TurtleNest", "TurtleNest");
  ui->maintainerEdit->setText(settings.value("maintainer_name", "").toString());
  ui->emailEdit->setText(settings.value("maintainer_email", "").toString());
}


MainWindow::~MainWindow()
{
  delete ui;
}

/* Bottom buttons bar */

void MainWindow::on_nextButton_clicked()
{
  ui->backButton->setVisible(true);
  auto last_index = ui->stackedWidget->count() - 1;
  auto new_index = ui->stackedWidget->currentIndex() + 1;

  // Hardcode skipping of the node creation page for MSGS packages
  BuildType build_type = get_selected_package_type();
  if (build_type == BuildType::MSGS && new_index == 2) {
    new_index++;
  }

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

  // Hardcode skipping of the node creation page for MSGS packages
  BuildType build_type = get_selected_package_type();
  if (build_type == BuildType::MSGS && new_index == 2) {
    new_index--;
  }

  ui->stackedWidget->setCurrentIndex(new_index);
  if (new_index == 0) {
    ui->backButton->setVisible(false);
  }
}


void MainWindow::on_createPackageButton_clicked()
{
  BuildType build_type = get_selected_package_type();

  PackageInfo pkg_info;
  pkg_info.workspace_path = ui->workspacePathEdit->text();
  pkg_info.package_name = ui->packageNameEdit->text();
  pkg_info.package_type = build_type;
  pkg_info.description = ui->descriptionEdit->toPlainText();
  pkg_info.maintainer = ui->maintainerEdit->text();

  // Row 0 is "No Licence", so it is not directly usable as the license.
  if (ui->licenseList->currentRow() != 0) {
    pkg_info.license = get_license();
  }

  if (!ui->emailEdit->text().trimmed().isEmpty()) {
    pkg_info.maintainer_email = ui->emailEdit->text();
  }

  QSettings settings("TurtleNest", "TurtleNest");
  settings.setValue("maintainer_name", ui->maintainerEdit->text());
  settings.setValue("maintainer_email", ui->emailEdit->text());

  QString launch_name = "";
  if (ui->checkboxCreateLaunch->isChecked()) {
    launch_name = ui->lineEditLaunchName->text();
  }

  QString params_file_name = "";
  if (ui->checkboxCreateParams->isChecked()) {
    params_file_name = ui->lineEditParamsName->text();
  }

  bool create_config = (params_file_name != "");
  bool create_node = ui->nodeTypeListWidget->currentRow() != 0;

  NodeType node_type;
  QString node_name;
  if (create_node){
    node_name = ui->nodeNameLineEdit->text();
    node_type = node_type_from_string(ui->nodeTypeListWidget->currentItem()->text());
  }

  try {
    create_package(pkg_info);

    if (create_node){
      bool composable_launch = node_type == NodeType::CPP_COMPOSABLE_NODE;
      create_launch_and_params(
        pkg_info,
        launch_name,
        params_file_name,
        node_name,
        composable_launch
      );
      //pkg_generator->create_node(node_name, node_type, create_config);
      // TODO: Handle the node generation

      // TODO: Catch the errors separately for each step.
      // TODO: Add instructions to post an issue if the generation fails.
    }
    else{
      create_launch_and_params(pkg_info, launch_name, params_file_name, "", false);
    }
    //pkg_creator.create_package();
  } catch (const std::runtime_error & error) {
    qCritical().noquote() << "Package Creation Failed: " << error.what();
    QMessageBox::critical(this, "Package Creation Failed", error.what());
    return;
  }

  qInfo() << "Package created successfully!";
  QString success_msg = "ROS 2 package '" + pkg_info.package_name +
    "' has been successfully created. You can now build the package.";

  QMessageBox::information(this, "Package Creation Successful", success_msg);
  accept();  // Close and set result as Accepted
}


/* PAGE 1 */

void MainWindow::on_browseButton_clicked()
{
  auto workspace_path = QFileDialog::getExistingDirectory(this, "Select Folder", "");

  if (workspace_path.isEmpty()) {
    return;
  }
  ui->workspacePathEdit->setText(workspace_path);
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


/* PAGE 2 */

void MainWindow::handle_package_type_changed(int /*id*/, bool checked)
{
  if (!checked) {
    return;
  }
  BuildType package_type = get_selected_package_type();
  update_package_type_page_ui(package_type);
}

BuildType MainWindow::get_selected_package_type()
{
  switch (package_type_group->checkedId()) {
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

void MainWindow::update_package_type_page_ui(BuildType package_type)
{
  ui->paramLaunchWidget->setVisible(true);

  // Update the list of node types
  std::unique_ptr<BaseNodeGenerator> node_generator = create_node_generator(package_type);
  std::vector<NodeType> supported_nodes = node_generator->get_supported_node_types();
  ui->nodeTypeListWidget->clear();
  ui->nodeTypeListWidget->addItem("No Node");
  for (NodeType & node_type: supported_nodes) {
    ui->nodeTypeListWidget->addItem(node_type_to_string(node_type));
  }
  ui->nodeTypeListWidget->setCurrentRow(0);

  // Set widget visibility based on the build type
  if (package_type == BuildType::MSGS) {
    ui->lineEditLaunchName->clear();
    ui->lineEditParamsName->clear();
    ui->checkboxCreateLaunch->setChecked(false);
    ui->checkboxCreateParams->setChecked(false);
    ui->paramLaunchWidget->setVisible(false);
  }
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


void MainWindow::on_launchNameInfoButton_clicked()
{
  show_tooltip(ui->launchNameInfoButton);
}


void MainWindow::on_paramsNameInfoButton_clicked()
{
  show_tooltip(ui->paramsNameInfoButton);
}

/* PAGE 3 */

void MainWindow::on_nodeTypeListWidget_currentItemChanged(
  QListWidgetItem * current,
  QListWidgetItem * /*previous*/)
{
  if (current) {
    int row = ui->nodeTypeListWidget->row(current);
    if (row == 0) { // No Node
      ui->nodeNameLineEdit->setEnabled(false);
      ui->nodeNameLineEdit->clear();
    } else {
      if (!ui->nodeNameLineEdit->isEnabled()) {
        ui->nodeNameLineEdit->setEnabled(true);
        ui->nodeNameLineEdit->setText(ui->packageNameEdit->text() + "_node");
      }
    }
  }
}


void MainWindow::on_nodeNameLineEdit_textEdited(const QString & arg1)
{
  autocorrect_line_edit(arg1, ui->nodeNameLineEdit);
}


void MainWindow::on_nodeNameLineEdit_editingFinished()
{
  if (ui->nodeNameLineEdit->text().length() < 1) {
    ui->nodeNameLineEdit->setText(ui->packageNameEdit->text() + "_node");
  }
}

void MainWindow::on_NodeNameInfoButton_clicked()
{
  show_tooltip(ui->NodeNameInfoButton);
}


/* PAGE 4 */

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


QString MainWindow::get_created_package_name()
{
  return ui->packageNameEdit->text();
}

void show_tooltip(QToolButton * button)
{
  QToolTip::showText(button->mapToGlobal(QPoint(16, 16)), button->toolTip());
}
