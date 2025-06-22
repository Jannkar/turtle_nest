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

#include "turtle_nest/package_xml_tools.h"
#include <QDir>
#include <QDebug>

// An editor to modify package.xml files for easy dependency adding
PackageXMLEditor::PackageXMLEditor(QString package_path)
{
  package_xml_path = QDir(package_path).filePath("package.xml");
  XMLError result = doc.LoadFile(package_xml_path.toStdString().c_str());
  if (result != XML_SUCCESS) {
    throw std::runtime_error("Failed to load package.xml");
  }

  // Check if root exists
  root = doc.RootElement();
  if (!root) {
    throw std::runtime_error("No root element in the package.xml file!");
  }

  // Check that the root is <package>
  if (std::string(root->Name()) != "package") {
    throw std::runtime_error("Root element is not <package> in the package.xml file!");
  }
}

// Adds any given dependency
void PackageXMLEditor::add_dependency(QString dependency, DependencyType depend_type)
{
  std::string depend_type_str = depend_type_to_string(depend_type);

  // Create the new element
  XMLElement * new_depend = doc.NewElement(depend_type_str.c_str());
  new_depend->SetText(dependency.toStdString().c_str());

  // Find the insertion point and insert the new dep
  XMLElement * insert_after = find_dep_insert_point(depend_type_str);
  root->InsertAfterChild(insert_after, new_depend);

  // Save the modified XML
  // Note: tinyxml2 does not preserve empty lines in the XML
  save_xml();
}

// Checks if dependency already exists
bool PackageXMLEditor::has_dependency(QString dependency, DependencyType depend_type)
{
  // Iterate through all depend_type elements under root
  std::string depend_type_str = depend_type_to_string(depend_type);
  const char * depend_cstr = depend_type_str.c_str();
  for (XMLElement * dependElem = root->FirstChildElement(depend_cstr); dependElem != nullptr;
    dependElem = dependElem->NextSiblingElement(depend_cstr))
  {
    const char * text = dependElem->GetText();
    if (text && std::string(text) == dependency.toStdString()) {
      return true;
    }
  }
  return false;
}


// Custom saving function to be used with our custom Printer, to preserve
// 2-spaces indentation instead of the default 4 in tinyxml2
void PackageXMLEditor::save_xml()
{
  FILE * file = fopen(package_xml_path.toStdString().c_str(), "w");
  if (file) {
    CustomXMLPrinter printer(file);
    doc.Print(&printer);
    fclose(file);
  } else {
    throw std::runtime_error("Failed to save package.xml");
  }
}

/**
 * @brief Finds the insertion point of a dependency, based on its type.
 * Insertion point will be at the end of the same type of depend_str, if
 * there already exists one. Otherwise we try to add it after the previous
 * dependency type, in order. If no other dependency type is found, will
 * give the insertion point to be after the license tag. If that also fails,
 * raises a runtime exception
 * @param depend_str: dependency type
 * @return insertion point
 */
XMLElement * PackageXMLEditor::find_dep_insert_point(std::string depend_str)
{
  // The insertion order of tags in the XML:
  QStringList tag_order = {"license", "buildtool_depend", "depend", "build_depend", "exec_depend", "member_of_group"};

  int idx = tag_order.indexOf(QString::fromStdString(depend_str));
  if (idx == -1) {
    std::string err_msg = "Unknown tag when trying to create a package.xml file: " + depend_str;
    throw std::runtime_error(err_msg);
  }

  // Find the last depend_type dependency tag to insert after
  XMLElement * insert_after = nullptr;
  for (XMLElement * dep = root->FirstChildElement(depend_str.c_str()); dep;
    dep = dep->NextSiblingElement(depend_str.c_str()))
  {
    insert_after = dep;
  }

  // If none found, check all tags with lower priority (i.e. with smaller index in the list)
  if (!insert_after) {
    for (int i = idx - 1; i >= 0; --i) {
      QString lower_priority_tag = tag_order[i];
      XMLElement * last_tag = nullptr;
      for (XMLElement * elem = root->FirstChildElement(lower_priority_tag.toStdString().c_str());
        elem; elem = elem->NextSiblingElement(lower_priority_tag.toStdString().c_str()))
      {
        last_tag = elem;
      }
      if (last_tag) {
        insert_after = last_tag;
        break;
      }
    }
  }
  if (!insert_after) {
    throw std::runtime_error(
            "Failed to insert a new dependency tag in to the package.xml. Update package.xml manually!");
  }
  return insert_after;
}

std::string depend_type_to_string(DependencyType depend_type)
{
  switch (depend_type) {
    case DependencyType::DEPEND: return "depend";
    case DependencyType::BUILD_DEPEND: return "build_depend";
    case DependencyType::EXEC_DEPEND: return "exec_depend";
    case DependencyType::BUILDTOOL_DEPEND: return "buildtool_depend";
    case DependencyType::MEMBER_OF_GROUP: return "member_of_group";
    default:
      throw std::runtime_error("Unknown dependency type");
  }
}
