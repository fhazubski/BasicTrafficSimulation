LIBS += -L$$PWD/../install/tslib/bin/ -ltslib

INCLUDEPATH += $$PWD/../install

HEADERS += \
    $$PWD/../install/tslib/include/api.h \
    $$PWD/../install/tslib/include/tslib_export.h \
    $$PWD/../install/tslib/include/types.h
