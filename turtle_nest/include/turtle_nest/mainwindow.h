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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QToolButton>


QT_BEGIN_NAMESPACE
namespace Ui {
  class MainWindow;
}
QT_END_NAMESPACE


void show_tooltip(QToolButton * button);

class MainWindow: public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget * parent = nullptr);
  ~MainWindow();

// Uncrustify wants to indent private slots after Jazzy distro, but not before Humble,
// leading to a conflict. Skip.
/* *INDENT-OFF* */
  private slots:
/* *INDENT-ON* */
  void on_nextButton_clicked();

  void on_backButton_clicked();

  void on_browseButton_clicked();

  void on_createPackageButton_clicked();

  void on_packageNameEdit_textEdited(const QString & arg1);

  void on_packageNameEdit_editingFinished();

  void on_checkboxCreateLaunch_toggled(bool checked);

  void change_package_type();

  void on_checkboxCpp_clicked();

  void on_checkboxPython_clicked();

  void on_lineEditNodeNameCpp_textEdited(const QString & arg1);

  void on_lineEditNodeNamePython_textEdited(const QString & arg1);

  void on_lineEditLaunchName_textEdited(const QString & arg1);

  void on_lineEditLaunchName_editingFinished();

  QString get_license();

  void on_emailEdit_textEdited(const QString & arg1);

  void on_emailEdit_editingFinished();

  void on_pkgNameInfoButton_clicked();

  void on_cppNodeNameInfoButton_clicked();

  void on_pythonNodeNameInfoButton_clicked();

  void on_launchNameInfoButton_clicked();

private:
  Ui::MainWindow * ui;
};

#endif // MAINWINDOW_H
