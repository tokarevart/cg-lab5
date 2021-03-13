QT += core gui opengl openglwidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    tiny_obj_loader.cpp

HEADERS += \
    camera.h \
    graphics-view-3d.h \
    helpers.h \
    mainwindow.h \
    mat.h \
    mesh.h \
    point-light.h \
    render.h \
    scene.h \
    sptalgs.h \
    sptops.h \
    tiny_obj_loader.h \
    vec.h \
    view.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
