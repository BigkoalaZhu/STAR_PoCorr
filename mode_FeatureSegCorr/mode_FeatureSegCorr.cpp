#include <QMouseEvent>
#include "SurfaceMeshPlugins.h"
#include "ModePluginDockWidget.h"
#include "mode_FeatureSegCorr.h"
#include "StarlabDrawArea.h"
#include <fstream>



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

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
void writeToCSVfile(QString name, Eigen::MatrixXd matrix)
{
	std::ofstream file(name.toStdString().c_str());
	file << matrix.format(CSVFormat);
	file.close();
}

void FeatureSegCorr::setrunCalcHKS()
{
	// Initial point cloud data to KDtree
	m1 = SurfaceMesh::safe_cast(document()->selectedModel());
	Eigen::Map<Eigen::Matrix3Xd> X((double *)(m1->vertex_coordinates().data()), 3, m1->n_vertices());
	Eigen::Map<Eigen::Matrix3Xd> dst_normals((double *)(m1->vertex_normals().data()), 3, m1->n_vertices());
	SurfaceMesh::VertexCoordinatesIterator vci(m1);
    NanoKDTree3<Vector3> kdtree(vci.begin(), vci.end());

	// Get Average distance
	double avg = 0;
	for (int i = 0; i < X.cols(); i++)
		avg += kdtree.closest_dist(X.col(i).data());
	avg = avg/X.cols();
	radius = avg*radius;

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
			int piece_r = 0;
			while(piece_r < piece_layer)
			{
				if(pieces(j,piece_r) == 0)
					break;
				piece_r++;
			}
			double tmpv = (4.0f/3.0f)*M_PI*pow(piece_r*radius/piece_layer,3)/piece_num;
			volume[i] = volume[i] + tmpv;
		}
	}
	
	// Calculate Laplace Operator
	int time_sq_num = 1024;
	double t = radius*radius/4;
	Eigen::MatrixXd Lp = Eigen::MatrixXd::Zero(X.cols(),X.cols());
	for (int j = 0; j < X.cols(); j++)
		for (int k = j; k < X.cols(); k++)
		{
			double dist = (X.col(j) - X.col(k)).squaredNorm();
			double l = (-1.0f/(pow(4*M_PI*t,1.5)*t))*(volume[k]/4.0f)*exp(dist/(4.0f*t));
			Lp(j,k) = l;
			Lp(k,j) = Lp(j,k);
		}

	// Calculate HKS k(x,x)
	QVector<Eigen::VectorXd> HKST;
	for (int i = 0; i < time_sq_num; i++)
	{
		double t = i + 1;
		Eigen::VectorXd HKS = Eigen::VectorXd::Zero(X.cols());
		writeToCSVfile("m.csv",LpT[i]);
		Eigen::EigenSolver<Eigen::MatrixXd> esL(LpT[i]);
		Eigen::VectorXd egval = esL.eigenvalues().col(0).real();
		Eigen::MatrixXd egvec = esL.eigenvectors().real();
		for (int j = 0; j < X.cols(); j++)
		{
			double h = 0;
			for (int n = 0; n < X.cols(); n++)
			{
				h += exp(-egval(n)*t)*egvec.col(n).dot(egvec.col(n));
			}
			HKS[j] = h;
		}
		HKST.push_back(HKS);
	}
}
