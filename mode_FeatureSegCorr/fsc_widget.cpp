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
	mode->connect(ui->CFF, SIGNAL(clicked()), SLOT(setrunCalcCFF()));
	mode->connect(ui->cff_2, SIGNAL(clicked()), SLOT(setrunCalcCFF2()));

	mode->connect(ui->radiusIncff, SIGNAL(textChanged(QString)), SLOT(setRadiusIncff(QString)));
	mode->connect(ui->Sigma, SIGNAL(textChanged(QString)), SLOT(setSigma(QString)));

	ui->radius->setText("30");
	ui->Maxt->setText("128");
	ui->radiusIncff->setText("3");
	ui->Sigma->setText("5");
	ui->colorize->setChecked(true);
	
	/////////////////////////////////////////////////////////Label
	mode->connect(ui->label_gn, SIGNAL(textChanged(QString)), SLOT(setlabel_gn(QString)));
	mode->connect(ui->label_ith, SIGNAL(valueChanged(int)), SLOT(setlabel_ith(int)));
	mode->connect(ui->label_confirm, SIGNAL(clicked()), SLOT(setlabel_gn_confirm()));
}

fsc_widget::~fsc_widget()
{
    delete ui;
}
