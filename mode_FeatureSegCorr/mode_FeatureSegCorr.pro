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
	APCluster.h \
	SelfTuningCluster\ClusterRotate.h \
	SelfTuningCluster\Evrot.h \
	SelfTuningCluster\Kmeans.h \
	SelfTuningCluster\SpectralClustering.h
	
SOURCES += \
    mode_FeatureSegCorr.cpp \
    fsc_widget.cpp \
	GetConvexHull.cpp \
	APCluster.cpp \
	SelfTuningCluster\ClusterRotate.cpp \
	SelfTuningCluster\Evrot.cpp \
	SelfTuningCluster\Kmeans.cpp \
	SelfTuningCluster\SpectralClustering.cpp
	
FORMS += \
    fsc_widget.ui
RESOURCES += \
	mode_FeatureSegCorr.qrc