win32: LIBS += -L$$PWD/../tsblib/build/lib_install/ -ltslib

INCLUDEPATH += $$PWD/../tsblib/build/include_install
DEPENDPATH += $$PWD/../tsblib/build/lib_install

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/../tsblib/build/lib_install/tslib.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/../tsblib/build/lib_install/libtslib.a

HEADERS += \
    $$PWD/../tsblib/build/include_install/tslib/api.h \
    $$PWD/../tsblib/build/include_install/tslib/tslib_export.h \
    $$PWD/../tsblib/build/include_install/tslib/types.h
