include($$[STARLAB])
include($$[SURFACEMESH])
include($$[NANOFLANN])
include($$[EIGEN])
StarlabTemplate(plugin)

Release: LIBS += -L 4pcs_1.3/ANN/ReleaseLib
Debug: LIBS += -L 4pcs_1.3/ANN/DebugLib

LIBS += -lANN

HEADERS += filter_4pcs.h
HEADERS += 4pcs_1.3/4pcs.h
SOURCES += filter_4pcs.cpp
SOURCES += 4pcs_1.3/4pcs.cc

