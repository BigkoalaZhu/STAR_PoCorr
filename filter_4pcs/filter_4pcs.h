#pragma once
#include "SurfaceMeshPlugins.h"
using namespace SurfaceMesh;

class filter_4pcs : public SurfaceMeshFilterPlugin{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "4pcs.plugin.starlab")
    Q_INTERFACES(FilterPlugin)

public:
    QString name() { return "4-points congruent sets for robust surface registration"; }
    void applyFilter(RichParameterSet*);
    void initParameters(RichParameterSet*);
};
