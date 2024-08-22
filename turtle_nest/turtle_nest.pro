QT       += core gui printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src/file_utils.cpp \
    src/generate_launch.cpp \
    src/generate_node.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    src/rospkgcreator.cpp \
    src/string_tools.cpp

HEADERS += \
    include/turtle_nest/build_type_enum.h \
    include/turtle_nest/file_utils.h \
    include/turtle_nest/generate_launch.h \
    include/turtle_nest/generate_node.h \
    include/turtle_nest/mainwindow.h \
    include/turtle_nest/rospkgcreator.h \
    include/turtle_nest/string_tools.h

FORMS += \
    src/mainwindow.ui

INCLUDEPATH += /usr/include
INCLUDEPATH += $$PWD/include

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    custom_theme.qss

RESOURCES += \
    resources.qrc
