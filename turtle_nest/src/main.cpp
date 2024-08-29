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
#include <QApplication>
#include <QFile>
#include <QTextStream>

int main(int argc, char * argv[])
{
  // Fix the problem with application not scaling correctly on scaled displays
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
    Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

  QApplication a(argc, argv);

  MainWindow w;
  w.setWindowTitle("Create a New Package");
  w.show();

  QFile file(":/stylesheets/custom_theme.qss");

  if (file.open(QFile::ReadOnly | QFile::Text)) {
    QTextStream stream(&file);
    a.setStyleSheet(stream.readAll());
  }

  return a.exec();
}
