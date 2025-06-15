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

#ifndef PACKAGE_XML_TOOLS_H
#define PACKAGE_XML_TOOLS_H

#include <QString>
#include <tinyxml2.h>

using namespace tinyxml2;


enum class DependencyType
{
  DEPEND,
  BUILD_DEPEND,
  EXEC_DEPEND
  // TEST_DEPEND and BUILD_EXPORT_DEPEND not needed
};

std::string depend_type_to_string(DependencyType depend_type);

class PackageXMLEditor {
public:
  PackageXMLEditor(QString package_path);
  void add_dependency(QString dependency, DependencyType depend_type);
  bool has_dependency(QString dependency, DependencyType depend_type);

private:
  QString package_xml_path;
  XMLDocument doc;
  XMLElement * root;
  void save_xml();
  XMLElement * find_dep_insert_point(std::string depend_str);
};


/**
 * @brief The CustomXMLPrinter class
 * Overrides the PrintSpace function to print 2 white spaces as indentation
 * instead of 4 white spaces, which is a ROS 2 practice.
 */
class CustomXMLPrinter: public tinyxml2::XMLPrinter {
public:
  CustomXMLPrinter(FILE * fp, bool compact = false) : XMLPrinter(fp, compact) {
  }
  void PrintSpace(int depth) override
  {
    for (int i = 0; i < depth; ++i) {
      Write("  ");
    }
  }
};

#endif // PACKAGE_XML_TOOLS_H
