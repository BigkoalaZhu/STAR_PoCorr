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
	double piece_radius = M_PI/4;
	int piece_layer = 3;
	int piece_num = 2*(M_PI/piece_radius)*(M_PI/piece_radius);
	Eigen::VectorXd volume = Eigen::VectorXd::Zero(X.cols());
	for (int i = 0; i < X.cols(); i++)
	{
		Eigen::Vector3d pith = X.col(i);
		Eigen::Vector3d n1ith = dst_normals.col(i);
		Eigen::Vector3d n2ith = Eigen::Vector3d::Zero();
		if(abs(n1ith[0])>1.e-4&&abs(n1ith[1])>1.e-4)
		{
			n2ith[0] = - 1/n1ith[0];
			n2ith[1] = 1/n1ith[1];
			n2ith[2] = 0;
		}
		else if(abs(n1ith[0]) <= 1.e-4)
		{
			n2ith[0] = 1;
			n2ith[1] = 0;
			n2ith[2] = 0;
		}
		else
		{
			n2ith[0] = 0;
			n2ith[1] = 1;
			n2ith[2] = 0;
		}
		n1ith.normalize();
		n2ith.normalize();
		Eigen::Vector3d n3ith = n1ith.cross(n2ith);
		//////////////////////////////////////////////////
		std::vector<size_t> pointinball = kdtree.ball_search<Eigen::Vector3d>(pith,radius);
		std::vector<double> pointinball_dist = kdtree.ball_search_dist<Eigen::Vector3d>(pith,radius);
		Eigen::MatrixXd pieces = Eigen::MatrixXd::Zero(piece_num,piece_layer);
		for (int j = 1; j < pointinball.size(); j++)
		{
			Eigen::Vector3d pij = X.col(pointinball[j]) - pith;
			pij.normalize();
			double alph = n1ith.dot(pij);
			double beta = n2ith.dot(pij);
			double cita = n3ith.dot(pij);
			alph = acos(alph);
			beta = acos(beta);
			cita = acos(cita);
			if(cita > M_PI/2)
				beta = 2*M_PI - beta;
			int index = floor(beta/piece_radius)*int(M_PI/piece_radius) + ceil(alph/piece_radius);
			pieces(index,floor(pointinball_dist[j]/(radius/piece_layer))) = 1;
		}
		for (int j = 0; j < piece_num; j++)
		{
			
		}
	}
}
