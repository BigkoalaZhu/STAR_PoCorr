#ifndef FSC_WIDGET_H
#define FSC_WIDGET_H

#include <QWidget>
#include "SurfaceMeshPlugins.h"

class FeatureSegCorr;

namespace Ui {
	class fsc_widget;
}

class fsc_widget : public QWidget
{
    Q_OBJECT

public:
    explicit fsc_widget(FeatureSegCorr * m = 0);
    ~fsc_widget();

private:
    Ui::fsc_widget *ui;
	FeatureSegCorr *mode;
};

#endif
