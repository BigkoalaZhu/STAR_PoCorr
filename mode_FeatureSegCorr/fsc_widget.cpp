#include "fsc_widget.h"
#include "ui_fsc_widget.h"
#include "mode_FeatureSegCorr.h"

fsc_widget::fsc_widget(FeatureSegCorr * m) : ui(new Ui::fsc_widget)
{
    ui->setupUi(this);
}

fsc_widget::~fsc_widget()
{
    delete ui;
}
