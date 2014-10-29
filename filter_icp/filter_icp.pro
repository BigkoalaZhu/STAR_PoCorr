include($$[STARLAB])
include($$[SURFACEMESH])
include($$[NANOFLANN])
include($$[EIGEN])
StarlabTemplate(plugin)

HEADERS += NanoKDTree3.h
HEADERS += RigidMotionEstimator.h
HEADERS += filter_icp.h
SOURCES += filter_icp.cpp

