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

#ifndef ADDNODEDIALOG_H
#define ADDNODEDIALOG_H

#include "turtle_nest/node_type_enum.h"
#include "turtle_nest/node_generators/base_node_generator.h"
#include "turtle_nest/packageinfo.h"
#include <QButtonGroup>
#include <QDialog>

namespace Ui {
  class AddNodeDialog;
}

class AddNodeDialog: public QDialog
{
  Q_OBJECT

public:
  explicit AddNodeDialog(QWidget * parent, const PackageInfo & package_info);
  ~AddNodeDialog();

  QString get_node_name();
  NodeType get_node_type();

// Uncrustify wants to indent private slots after Jazzy distro, but not before Humble,
// leading to a conflict. Skip.
/* *INDENT-OFF* */
  private slots:
/* *INDENT-ON* */
  void on_nodeNameEdit_textEdited(const QString & arg1);

private:
  Ui::AddNodeDialog * ui;
  PackageInfo pkg_info;
  std::unique_ptr < BaseNodeGenerator > node_generator;
  std::vector < NodeType > supported_node_types;
  QButtonGroup * node_button_group;
  void populate_node_types();
};

#endif // ADDNODEDIALOG_H
