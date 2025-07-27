QT       += core gui printsupport svgwidgets concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src/addnodedialog.cpp \
    src/file_utils.cpp \
    src/generate_cmake.cpp \
    src/generate_launch.cpp \
    src/generate_msgs_pkg.cpp \
    src/generate_params.cpp \
    src/generate_setup_py.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    src/modify_existing_pkg.cpp \
    src/package_generators/cpp_package_generator.cpp \
    src/package_generators/mixed_cpp_python_package_generator.cpp \
    src/package_generators/package_generator_factory.cpp \
    src/package_generators/python_package_generator.cpp \
    src/package_xml_tools.cpp \
    src/packageinfoform.cpp \
    src/packageswindow.cpp \
    src/rospkgcreator.cpp \
    src/string_tools.cpp

HEADERS += \
    include/turtle_nest/build_type_enum.h \
    include/turtle_nest/file_utils.h \
    include/turtle_nest/generate_cmake.h \
    include/turtle_nest/generate_launch.h \
    include/turtle_nest/generate_msgs_pkg.h \
    include/turtle_nest/generate_setup_py.h \
    include/turtle_nest/mainwindow.h \
    include/turtle_nest/modify_existing_pkg.h \
    include/turtle_nest/node_type_enum.h \
    include/turtle_nest/package_generators/cpp_package_generator.h \
    include/turtle_nest/package_generators/base_package_generator.h \
    include/turtle_nest/package_generators/mixed_cpp_python_package_generator.h \
    include/turtle_nest/package_generators/package_generator_factory.h \
    include/turtle_nest/package_generators/python_package_generator.h \
    include/turtle_nest/package_xml_tools.h \
    include/turtle_nest/packageinfo.h \
    include/turtle_nest/rospkgcreator.h \
    include/turtle_nest/string_tools.h \
    include/turtle_nest/generate_params.h \
    include/turtle_nest/packageswindow.h \
    include/turtle_nest/packageinfoform.h \
    include/turtle_nest/addnodedialog.h

FORMS += \
    src/addnodedialog.ui \
    src/mainwindow.ui \
    src/packageinfoform.ui \
    src/packageswindow.ui


INCLUDEPATH += /usr/include
INCLUDEPATH += $$PWD/include
INCLUDEPATH += /usr/include/pybind11

LIBS += -ltinyxml2

# Add Python3 to support script running with pybind11
PYVERSION = $$system(python3 -c \"import sys;v=sys.version_info;print(str(v[0])+\'.\'+str(v[1]))\")
message("Python version detected: $$PYVERSION")
INCLUDEPATH += /usr/include/python$$PYVERSION
LIBS += -lpython$$PYVERSION

# Custom post-build step to copy a python script
PYTHON_SCRIPT = $$PWD/src/python_file_utils.py
SCRIPT_DEST_DIR = $$OUT_PWD/lib/$$TARGET
QMAKE_POST_LINK += mkdir -p $$SCRIPT_DEST_DIR$$escape_expand(\\n\\t)
QMAKE_POST_LINK += cp $$PYTHON_SCRIPT $$SCRIPT_DEST_DIR/

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    custom_theme.qss

RESOURCES += \
    resources.qrc
