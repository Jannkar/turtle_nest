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

#include "turtle_nest/packageinfoform.h"
#include "turtle_nest/modify_existing_pkg.h"
#include "turtle_nest/node_type_enum.h"
#include "turtle_nest/modify_existing_pkg.h"
#include "turtle_nest/addnodedialog.h"
#include "ui_packageinfoform.h"

#include <QLabel>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsSvgItem>
#include <QSvgRenderer>
#include <QSvgWidget>
#include <QDebug>
#include <QMessageBox>


PackageInfoForm::PackageInfoForm(QWidget * parent)
: QWidget(parent)
  , ui(new Ui::PackageInfoForm)
{
  ui->setupUi(this);
  this->layout()->setAlignment(Qt::AlignTop);
  this->layout()->setSpacing(24);
  ui->pkgTypeLogosLayout->setAlignment(Qt::AlignLeft);

  // qss styling
  ui->packageTitleLabel->setObjectName("header");
  ui->packageDescLabel->setObjectName("description");
  ui->NodesLabel->setObjectName("header2");
  ui->launchLabel->setObjectName("header2");
  ui->paramsLabel->setObjectName("header2");

  // Package details
  ui->versionTitleLabel->setObjectName("package_details_title_label");
  ui->licenseTitleLabel->setObjectName("package_details_title_label");

  ui->versionLabel->setObjectName("package_details_value");
  ui->licenseLabel->setObjectName("package_details_value");

  // Add Package Type Logos
  const int language_logo_size = 32;
  cpp_svg = createScaledSvg(":/images/img/cpp_logo.svg", language_logo_size);
  cpp_svg->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  python_svg = createScaledSvg(":/images/img/python-logo-generic.svg", language_logo_size);
  python_svg->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  msgs_svg = createScaledSvg(":/images/img/msgs.svg", 28);
  msgs_svg->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  ui->pkgTypeLogosLayout->addWidget(cpp_svg);
  ui->pkgTypeLogosLayout->addWidget(python_svg);
  ui->pkgTypeLogosLayout->addWidget(msgs_svg);

  // Nodes
  ui->noNodesLabel->hide();
  ui->notBuiltWarnLabel->hide();
  ui->NodesListWidget->setDisabled(true);   // Completely disables interaction

  // Launch
  ui->noLaunchFilesLabel->hide();
  ui->launchListWidget->hide();
  ui->launchListWidget->setDisabled(true);

  // Params
  ui->noParamFilesLabel->hide();
  ui->paramsListWidget->hide();
  ui->paramsListWidget->setDisabled(true);

  // Create "Add new" -buttons
  QIcon add_node_icon(":/images/img/plus-line-icon.svg");
  ui->addNodeButton->setIcon(add_node_icon);
  ui->addNodeButton->setIconSize(QSize(16, 16));
}

PackageInfoForm::~PackageInfoForm()
{
  delete ui;
}

void PackageInfoForm::updatePackageInfo(PackageInfo pkg_info)
{
  current_package_info = pkg_info;
  ui->packageTitleLabel->setText(pkg_info.package_name);
  ui->packageDescLabel->setText(pkg_info.description);
  ui->versionLabel->setText(pkg_info.version);
  ui->licenseLabel->setText(pkg_info.license);

  // Set package language logos
  python_svg->hide();
  cpp_svg->hide();
  msgs_svg->hide();
  ui->addNodeButton->hide();

  // Set labels and widgets back to defaults
  ui->NodesLabel->hide();
  ui->NodesListWidget->hide();
  ui->NodesListWidget->clear();
  ui->notBuiltWarnLabel->hide();
  ui->noNodesLabel->hide();

  ui->launchLabel->hide();
  ui->launchListWidget->hide();
  ui->launchListWidget->clear();
  ui->noLaunchFilesLabel->hide();

  ui->paramsLabel->hide();
  ui->paramsListWidget->hide();
  ui->paramsListWidget->clear();
  ui->noParamFilesLabel->hide();

  switch (pkg_info.package_type) {
    case BuildType::CPP:
      updateNodesLaunchParams(pkg_info);
      cpp_svg->show();
      ui->addNodeButton->show();
      break;
    case BuildType::PYTHON:
      updateNodesLaunchParams(pkg_info);
      python_svg->show();
      ui->addNodeButton->show();
      break;
    case BuildType::CPP_AND_PYTHON:
      updateNodesLaunchParams(pkg_info);
      cpp_svg->show();
      python_svg->show();
      ui->addNodeButton->show();
      break;
    case BuildType::MSGS:
      msgs_svg->show();
      break;
    default:
      // No default icon or behavior selected for unknown packages
      break;
  }
}

void PackageInfoForm::updateNodesLaunchParams(PackageInfo pkg_info)
{
  // Update Nodes section
  ui->NodesLabel->show();
  ui->launchLabel->show();
  ui->paramsLabel->show();

  try {
    QStringList executables = list_executables(pkg_info.workspace_path, pkg_info.package_name);
    if (executables.empty()) {
      ui->noNodesLabel->show();
    } else {
      for (QString & executable : executables) {
        ui->NodesListWidget->addItem(executable);
      }
      adjustListWidgetHeight(ui->NodesListWidget);
      ui->NodesListWidget->show();
    }
  } catch (const std::runtime_error & error) {
    ui->notBuiltWarnLabel->show();
  }

  // Update Launch files section
  QStringList launch_files = list_files(QDir(pkg_info.package_path).filePath("launch"));
  if (launch_files.empty()) {
    ui->noLaunchFilesLabel->show();
  } else {
    for (QString & launch_file : launch_files) {
      ui->launchListWidget->addItem(launch_file);
      adjustListWidgetHeight(ui->launchListWidget);
    }
    ui->launchListWidget->show();
  }

  // Update Param files section
  QStringList param_files = list_files(QDir(pkg_info.package_path).filePath("config"));
  if (param_files.empty()) {
    ui->noParamFilesLabel->show();
  } else {
    for (QString & param_file : param_files) {
      ui->paramsListWidget->addItem(param_file);
      adjustListWidgetHeight(ui->paramsListWidget);
    }
    ui->paramsListWidget->show();
  }
}

QSvgWidget * PackageInfoForm::createScaledSvg(QString path, int targetHeight)
{
  QSvgWidget * svg = new QSvgWidget(this);
  svg->load(QString(path));
  QSize naturalSize = svg->renderer()->defaultSize();
  double aspectRatio = static_cast<double>(naturalSize.width()) / naturalSize.height();

  // Calculate the corresponding width to maintain the aspect ratio.
  int targetWidth = static_cast<int>(targetHeight * aspectRatio);
  svg->setFixedSize(targetWidth, targetHeight);
  return svg;
}


void adjustListWidgetHeight(QListWidget * listWidget)
{
  int totalHeight = 0;

  for (int i = 0; i < listWidget->count(); ++i) {
    totalHeight += listWidget->sizeHintForRow(i);
  }
  listWidget->setFixedHeight(totalHeight + 2 * listWidget->frameWidth());
}

void PackageInfoForm::on_addNodeButton_clicked()
{
  AddNodeDialog dialog(this, current_package_info);

  if (dialog.exec() == QDialog::Accepted) {
    QString node_name = dialog.get_node_name();
    NodeType node_type = dialog.get_node_type();
    try {
      add_node(node_name, node_type, current_package_info);
    } catch (const std::runtime_error & error) {
      qCritical().noquote() << "Node Creation Failed: " << error.what();
      QMessageBox::critical(this, "Node Creation Failed", error.what());
      return;
    }

    QString success_msg = "Node " + node_name +
      " added successfully! \nRebuild the package to see the newly created node in the nodes list.";
    QMessageBox::information(this, "Node Creation Successful", success_msg);

    updatePackageInfo(current_package_info);
  }
}
