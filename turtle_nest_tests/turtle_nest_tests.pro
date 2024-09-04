include(gtest_dependency.pri)

QT       += core gui printsupport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TEMPLATE = app
CONFIG += console c++17
CONFIG += thread

SOURCES += \
        main.cpp \
        tst_file_utils.cpp \
        tst_generate_node.cpp \
        tst_ros_pkg_creator.cpp \
        ../turtle_nest/src/file_utils.cpp \
        ../turtle_nest/src/generate_node.cpp \
        ../turtle_nest/src/rospkgcreator.cpp \
        ../turtle_nest/src/generate_cmake.cpp \
        ../turtle_nest/src/generate_launch.cpp \
        ../turtle_nest/src/generate_params.cpp \
        ../turtle_nest/src/generate_setup_py.cpp \
        ../turtle_nest/src/string_tools.cpp

INCLUDEPATH += ../turtle_nest/include/
