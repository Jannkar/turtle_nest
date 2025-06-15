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

class PackageInfoForm: public QWidget
{
  Q_OBJECT

public:
  explicit PackageInfoForm(QWidget * parent = nullptr);
  ~PackageInfoForm();
  void updatePackageInfo(PackageInfo pkg_info);
  QSvgWidget * createScaledSvg(QString path, int targetHeight);
  void updateNodesLaunchParams(PackageInfo pkg_info);

// Uncrustify wants to indent private slots after Jazzy distro, but not before Humble,
// leading to a conflict. Skip.
/* *INDENT-OFF* */
  private slots:
/* *INDENT-ON* */
  void on_addNodeButton_clicked();

private:
  Ui::PackageInfoForm * ui;
  QSvgWidget * cpp_svg;
  QSvgWidget * python_svg;
  QSvgWidget * msgs_svg;
  PackageInfo current_package_info;
};

void adjustListWidgetHeight(QListWidget * listWidget);

#endif // PACKAGEINFOFORM_H
