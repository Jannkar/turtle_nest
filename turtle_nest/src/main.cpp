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

#include "turtle_nest/packageswindow.h"
#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QFontDatabase>

void set_fonts();

int main(int argc, char * argv[])
{
  // Fix the problem with application not scaling correctly on scaled displays
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
    Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

  QApplication a(argc, argv);
  set_fonts();

  // This doesn't work for some reason.
  //QIcon appIcon(":/images/img/turtle_nest_app_icon.png");
  //a.setWindowIcon(appIcon);

  PackagesWindow w;
  w.setWindowTitle("Turtle Nest");
  w.show();

  QFile file(":/stylesheets/custom_theme.qss");
  if (file.open(QFile::ReadOnly | QFile::Text)) {
    QTextStream stream(&file);
    a.setStyleSheet(stream.readAll());
  }

  return a.exec();
}


void set_fonts()
{
  // iterate over every .ttf in the ":/fonts" resource directory
  QDir fontDir(":/fonts/fonts/Ubuntu_Sans/static");
  QStringList fontFiles = fontDir.entryList(QStringList{"*.ttf"}, QDir::Files);

  for (const QString & fileName : fontFiles) {
    const QString path = fontDir.absoluteFilePath(fileName);
    QFontDatabase::addApplicationFont(path);
  }
  QFont font = QFont("Ubuntu Sans");
  font.setPointSize(11);
  QApplication::setFont(font);
}
