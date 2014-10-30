#include <QMouseEvent>
#include "SurfaceMeshPlugins.h"
#include "ModePluginDockWidget.h"
#include "mode_FeatureSegCorr.h"
#include "StarlabDrawArea.h"



FeatureSegCorr::FeatureSegCorr()
{
    this->widget = NULL;
}

void FeatureSegCorr::create()
{
    if(!widget)
    {
        ModePluginDockWidget * dockwidget = new ModePluginDockWidget("FeatureSegCorr", mainWindow());
        widget = new fsc_widget(this);
        dockwidget->setWidget(widget);
        mainWindow()->addDockWidget(Qt::RightDockWidgetArea,dockwidget);
    }
    update();
}

void FeatureSegCorr::update()
{

}

void FeatureSegCorr::decorate()
{
    
}
