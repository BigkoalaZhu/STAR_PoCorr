#include <QMouseEvent>
#include "SurfaceMeshPlugins.h"
#include "ModePluginDockWidget.h"
#include "mode_FeatureSegCorr.h"
#include "StarlabDrawArea.h"



FeatureSegCorr::FeatureSegCorr()
{
    this->widget = NULL;
	colorize_flag = false;
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

void FeatureSegCorr::setColorize(int c)
{
	colorize_flag = c;
}

void FeatureSegCorr::setRadius(QString r)
{
	radius = r.toDouble();
}

void FeatureSegCorr::setrunCalcHKS()
{
	// Initial point cloud data to KDtree
	m1 = SurfaceMesh::safe_cast(document()->selectedModel());
	Eigen::Map<Eigen::Matrix3Xd> X((double *)(m1->vertex_coordinates().data()), 3, m1->n_vertices());
	Eigen::Map<Eigen::Matrix3Xd> dst_normals((double *)(m1->vertex_normals().data()), 3, m1->n_vertices());
	SurfaceMesh::VertexCoordinatesIterator vci(m1);
    NanoKDTree3<Vector3> kdtree(vci.begin(), vci.end());

	// Do the ball search
	Eigen::VectorXd volume = Eigen::VectorXd::Ones(X.cols());
	for (int i = 0; i < X.cols(); i++)
	{
		Eigen::Vector3d pith = X.col(i);
		std::vector<size_t> pointinball = kdtree.ball_search<Eigen::Vector3d>(pith,radius);

	}
}
