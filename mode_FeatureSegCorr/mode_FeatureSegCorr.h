#pragma once
#include "StarlabDrawArea.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "fsc_widget.h"
#include "NanoKDTree3.h"

enum SELECT_MODE{ADD, REMOVE, NONE};

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
	void drawWithNames();

    /// User interactions

private:
    fsc_widget * widget;
	bool colorize_flag;
    double radius;
	QVector<Eigen::VectorXd> HKST;
	int time_sq_num;
	Eigen::VectorXd volume;
	Eigen::MatrixXd Lp;
	double avg;
	double rcff;
	int sigma;

    Model *m1;
	////////////////////////////////////label
	int gn;
	int curgn;
	QVector<QSet<int>> group;

	bool endSelection(const QPoint& p);

	Vector3VertexProperty points;
    Vector3FaceProperty fnormals;

	QPoint cursorPos;
	int brushSize;
	SELECT_MODE selectMode;
	bool isDrawBrushSize;
	bool wheelEvent(QWheelEvent *);
    bool mouseMoveEvent(QMouseEvent*);

public slots:
	void setColorize(int);
	void setRadius(QString);
	void setrunCalcHKS();
	void setMaxt(QString);
	void display_t(int);
	void setrunCalcCFF();
	void setrunCalcCFF2();
	void setRadiusIncff(QString);
	void setSigma(QString);
	/////////////////////////////////////label
	void setlabel_gn(QString);
	void setlabel_ith(int);
	void setlabel_gn_confirm();
};


