include($$[STARLAB])
include($$[SURFACEMESH])
include($$[EIGEN])
include($$[NANOFLANN])
StarlabTemplate(plugin)

HEADERS += \
    mode_FeatureSegCorr.h \
    fsc_widget.h \
	NanoKDTree3.h
SOURCES += \
    mode_FeatureSegCorr.cpp \
    fsc_widget.cpp
FORMS += \
    fsc_widget.ui
RESOURCES += \
	mode_FeatureSegCorr.qrc