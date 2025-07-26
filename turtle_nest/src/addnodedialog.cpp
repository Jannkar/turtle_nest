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

#include "turtle_nest/addnodedialog.h"
#include "turtle_nest/node_type_enum.h"
#include <turtle_nest/package_generators/base_package_generator.h>
#include "turtle_nest/package_generators/package_generator_factory.h"
#include "turtle_nest/string_tools.h"
#include "ui_addnodedialog.h"
#include <QPushButton>
#include <QDebug>
#include <QDir>
#include <QRadioButton>
#include <turtle_nest/packageinfo.h>


AddNodeDialog::AddNodeDialog(QWidget * parent, const PackageInfo & package_info)
: QDialog(parent)
  , ui(new Ui::AddNodeDialog)
  , pkg_info(package_info)
{
  ui->setupUi(this);
  setWindowTitle("Add New Node");
  ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);

  // Setup the group box for different node types
  node_button_group = new QButtonGroup(this);
  node_button_group->setExclusive(true);
  pkg_generator = create_package_generator(pkg_info.package_type);
  supported_node_types = pkg_generator->get_supported_node_types();
  populate_node_types();

  // Make the dialog window to have a smallest size possible, after hiding elements.
  this->layout()->activate();
  this->resize(this->minimumSizeHint());
  this->setFixedSize(this->size());
}

AddNodeDialog::~AddNodeDialog()
{
  delete ui;
}

void AddNodeDialog::populate_node_types()
{
  for (NodeType node_type : supported_node_types) {
    QString type = node_type_to_string(node_type);
    QRadioButton * button = new QRadioButton(type, ui->nodeTypeBox);
    ui->nodeTypeBox->layout()->addWidget(button);
    node_button_group->addButton(button);
  }

  // Set the first one checked
  QList<QAbstractButton *> buttons = node_button_group->buttons();
  if (!buttons.isEmpty()) {
    buttons.first()->setChecked(true);
  }
}

void AddNodeDialog::on_nodeNameEdit_textEdited(const QString & arg1)
{
  autocorrect_line_edit(arg1, ui->nodeNameEdit);
  if (ui->nodeNameEdit->text().length() <= 1) {
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
  } else {
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
  }
}

QString AddNodeDialog::get_node_name()
{
  return ui->nodeNameEdit->text();
}

NodeType AddNodeDialog::get_node_type()
{
  QAbstractButton * selected = node_button_group->checkedButton();
  if (!selected) {
    throw std::logic_error("Node was not selected. This shouldn't be possible.");
  }
  QString text = selected->text();
  qDebug() << "Selected radio button text:" << text;
  return node_type_from_string(text);
}
