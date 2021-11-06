LIBS += -L../install/tslib/bin/ -ltslib

INCLUDEPATH += ../install

# This could be required for Windows
#win32:!win32-g++: PRE_TARGETDEPS += ../install/tslib/bin/tslib.lib

HEADERS += \
    ../install/tslib/include/api.h \
    ../install/tslib/include/tslib_export.h \
    ../install/tslib/include/types.h
