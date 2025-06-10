#ifndef PACKAGE_XML_TOOLS_H
#define PACKAGE_XML_TOOLS_H

#include <QString>
#include <tinyxml2.h>

using namespace tinyxml2;


enum class DependencyType{
    DEPEND,
    BUILD_DEPEND,
    EXEC_DEPEND
    // TEST_DEPEND and BUILD_EXPORT_DEPEND not needed
};

std::string depend_type_to_string(DependencyType depend_type);

class PackageXMLEditor{
public:
    PackageXMLEditor(QString package_path);
    void add_dependency(QString dependency, DependencyType depend_type);
    bool has_dependency(QString dependency, DependencyType depend_type);
private:
    QString package_xml_path;
    XMLDocument doc;
    XMLElement* root;
    void save_xml();
    XMLElement* find_dep_insert_point(std::string depend_str);
};


/**
 * @brief The CustomXMLPrinter class
 * Overrides the PrintSpace function to print 2 white spaces as indentation
 * instead of 4 white spaces, which is a ROS 2 practice.
 */
class CustomXMLPrinter : public tinyxml2::XMLPrinter {
public:
    CustomXMLPrinter(FILE* fp, bool compact = false) : XMLPrinter(fp, compact) {}
    void PrintSpace(int depth) override {
        for( int i=0; i<depth; ++i ) {
            Write( "  " );
        }
    }
};

#endif // PACKAGE_XML_TOOLS_H
