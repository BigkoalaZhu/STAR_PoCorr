#include "fsc_widget.h"
#include "ui_fsc_widget.h"
#include "mode_FeatureSegCorr.h"

fsc_widget::fsc_widget(FeatureSegCorr * m) : ui(new Ui::fsc_widget)
{
    ui->setupUi(this);

	this->mode = m;
	ui->changet->setRange(1,128);

	mode->connect(ui->colorize, SIGNAL(stateChanged(int)), SLOT(setColorize(int)));
	mode->connect(ui->radius, SIGNAL(textChanged(QString)), SLOT(setRadius(QString)));
	mode->connect(ui->Maxt, SIGNAL(textChanged(QString)), SLOT(setMaxt(QString)));
	mode->connect(ui->calcHKS, SIGNAL(clicked()), SLOT(setrunCalcHKS()));
	mode->connect(ui->changet, SIGNAL(valueChanged(int)), SLOT(display_t(int)));

	ui->radius->setText("5");
	ui->Maxt->setText("128");
	ui->colorize->setChecked(true);
	
}

fsc_widget::~fsc_widget()
{
    delete ui;
}
