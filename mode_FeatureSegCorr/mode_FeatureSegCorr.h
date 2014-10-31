#pragma once
#include "StarlabDrawArea.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "fsc_widget.h"
#include "NanoKDTree3.h"

class FeatureSegCorr : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "fcs.plugin.starlab")
    Q_INTERFACES(ModePlugin)

public:
    FeatureSegCorr();

    QIcon icon(){ return QIcon(":/fsc.png"); }

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();
	void update();

    /// User interactions

private:
    fsc_widget * widget;
	bool colorize_flag;
    double radius;
    Model *m1;
	Model *m2;

public slots:
	void setColorize(int);
	void setRadius(QString);
	void setrunCalcHKS();
};


