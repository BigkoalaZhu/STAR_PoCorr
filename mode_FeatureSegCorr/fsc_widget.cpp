#include "fsc_widget.h"
#include "ui_fsc_widget.h"
#include "mode_FeatureSegCorr.h"

fsc_widget::fsc_widget(FeatureSegCorr * m) : ui(new Ui::fsc_widget)
{
    ui->setupUi(this);

	this->mode = m;
	mode->connect(ui->colorize, SIGNAL(stateChanged(int)), SLOT(setColorize(int)));
	mode->connect(ui->radius, SIGNAL(textEdited(QString)), SLOT(setRadius(QString)));
	mode->connect(ui->calcHKS, SIGNAL(clicked()), SLOT(setrunCalcHKS()));
}

fsc_widget::~fsc_widget()
{
    delete ui;
}
