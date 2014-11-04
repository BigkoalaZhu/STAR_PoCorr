include($$[STARLAB])
include($$[SURFACEMESH])
include($$[EIGEN])
include($$[NANOFLANN])
include($$[QHULL])
StarlabTemplate(plugin)

HEADERS += \
    mode_FeatureSegCorr.h \
    fsc_widget.h \
	NanoKDTree3.h \
	GetConvexHull.h \
	GeoDrawObjects.h \
	APCluster.h
SOURCES += \
    mode_FeatureSegCorr.cpp \
    fsc_widget.cpp \
	GetConvexHull.cpp \
	APCluster.cpp
FORMS += \
    fsc_widget.ui
RESOURCES += \
	mode_FeatureSegCorr.qrc