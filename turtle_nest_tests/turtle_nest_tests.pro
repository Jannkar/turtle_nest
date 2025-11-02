include(gtest_dependency.pri)

QT       += core gui printsupport concurrent
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TEMPLATE = app
CONFIG += console c++17
CONFIG += thread

SOURCES += \
        main.cpp \
        test_utils.cpp \
        tst_file_utils.cpp \
        tst_modify_existing_pkg.cpp \
        tst_package_xml_tools.cpp \
        tst_package_creation.cpp \
        ../turtle_nest/src/file_utils.cpp \
        ../turtle_nest/src/generate_launch.cpp \
        ../turtle_nest/src/generate_params.cpp \
        ../turtle_nest/src/modify_existing_pkg.cpp \
        ../turtle_nest/src/node_generators/cpp_node_generator.cpp \
        ../turtle_nest/src/node_generators/mixed_cpp_python_node_generator.cpp \
        ../turtle_nest/src/node_generators/node_generator_factory.cpp \
        ../turtle_nest/src/node_generators/python_node_generator.cpp \
        ../turtle_nest/src/package_generators/base_package_generator.cpp \
        ../turtle_nest/src/package_generators/cpp_package_generator.cpp \
        ../turtle_nest/src/package_generators/create_package.cpp \
        ../turtle_nest/src/package_generators/mixed_package_generator.cpp \
        ../turtle_nest/src/package_generators/msgs_package_generator.cpp \
        ../turtle_nest/src/package_generators/python_package_generator.cpp \
        ../turtle_nest/src/package_xml_tools.cpp \
        ../turtle_nest/src/string_tools.cpp


LIBS += -ltinyxml2

# Define path for the fixtures
FIXTURES_DIR = $$PWD/../turtle_nest_tests/fixtures
DEFINES += FIXTURES_PATH=\\\"$$FIXTURES_DIR\\\"

# Add Python3 to support script running with pybind11
PYVERSION = $$system(python3 -c \"import sys;v=sys.version_info;print(str(v[0])+\'.\'+str(v[1]))\")
message("Python version detected: $$PYVERSION")
INCLUDEPATH += /usr/include/python$$PYVERSION
LIBS += -lpython$$PYVERSION

# Custom post-build step to copy a python script
PYTHON_SCRIPT = $$PWD/../turtle_nest/src/python_file_utils.py
SCRIPT_DEST_DIR = $$OUT_PWD/lib/$$TARGET
QMAKE_POST_LINK += mkdir -p $$SCRIPT_DEST_DIR$$escape_expand(\\n\\t)
QMAKE_POST_LINK += cp $$PYTHON_SCRIPT $$SCRIPT_DEST_DIR/

INCLUDEPATH += ../turtle_nest/include/

HEADERS += \
    test_utils.h
