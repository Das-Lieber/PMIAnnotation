QT += widgets gui opengl

CONFIG += c++11

DEFINES += QT_DEPRECATED_WARNINGS

FORMS += \
    Dialogs/DatumInput.ui \
    Dialogs/DiamensionInput.ui \
    Dialogs/TolBaseInput.ui \
    Dialogs/ToleranceInput.ui \
    MainWindow.ui

HEADERS += \
    Dialogs/DatumInput.h \
    Dialogs/DiamensionInput.h \
    Dialogs/TolBaseInput.h \
    Dialogs/ToleranceInput.h \
    Label/Label_Angle.h \
    Label/Label_Datum.h \
    Label/Label_Diameter.h \
    Label/Label_Length.h \
    Label/Label_PMI.h \
    Label/Label_Radius.h \
    Label/Label_Taper.h \
    Label/Label_Tolerance.h \
    MainWindow.h \
    OCCTool/AIS_DraftPoint.h \
    OCCTool/AIS_DraftShape.hxx \
    OCCTool/GeneralTools.h \
    OCCTool/OccWidget.h \
    OCCTool/PMIModel.h \
    OCCTool/pca.h \
    TolStringInfo.h

SOURCES += \
    Dialogs/DatumInput.cpp \
    Dialogs/DiamensionInput.cpp \
    Dialogs/TolBaseInput.cpp \
    Dialogs/ToleranceInput.cpp \
    Label/Label_Angle.cpp \
    Label/Label_Datum.cpp \
    Label/Label_Diameter.cpp \
    Label/Label_Length.cpp \
    Label/Label_PMI.cpp \
    Label/Label_Radius.cpp \
    Label/Label_Taper.cpp \
    Label/Label_Tolerance.cpp \
    MainWindow.cpp \
    OCCTool/AIS_DraftPoint.cpp \
    OCCTool/GeneralTools.cpp \
    OCCTool/OccWidget.cpp \
    OCCTool/PMIModel.cpp \
    OCCTool/pca.cpp \
    main.cpp

DESTDIR = $$PWD/bin

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

OCCTLIB_PATH = D:/OpenCasCade

win32 {
    contains(QT_ARCH, x86_64){
        contains(QMAKE_MSC_VER, 1916){
            INCLUDEPATH += $$OCCTLIB_PATH/inc
            LIBS += $$OCCTLIB_PATH/lib/*.lib
            message("using msvc")
        }else{
            mingw{
                INCLUDEPATH += $$OCCTLIB_PATH/inc
                LIBS += $$OCCTLIB_PATH/lib/lib*.a
                message("using mingw")
            }else{
                message("wrong kit config")
            }
        }
    }else{
        message("wrong system version")
    }
}

