LIBS += -L$$PWD/../install/tslib/bin/ -ltslib

INCLUDEPATH += $$PWD/../install
DEPENDPATH += $$PWD/../install

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/../install/tslib/bin/tslib.lib
else:unix|win32-g++: PRE_TARGETDEPS += $$PWD/../install/tslib/bin/libtslib.a

HEADERS += \
    $$PWD/../install/tslib/include/api.h \
    $$PWD/../install/tslib/include/tslib_export.h \
    $$PWD/../install/tslib/include/types.h
