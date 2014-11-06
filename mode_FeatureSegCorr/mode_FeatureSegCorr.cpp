#include <QMouseEvent>
#include "SurfaceMeshPlugins.h"
#include "ModePluginDockWidget.h"
#include "mode_FeatureSegCorr.h"
#include "StarlabDrawArea.h"
#include "GeoDrawObjects.h"
#include "GetConvexHull.h"
#include "APCluster.h"
#include "SelfTuningCluster\SpectralClustering.h"
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

void FeatureSegCorr::setMaxt(QString t)
{
	time_sq_num = t.toDouble();
}

void FeatureSegCorr::setRadiusIncff(QString r)
{
	rcff = r.toDouble();
}
void FeatureSegCorr::setSigma(QString s)
{
	sigma = s.toInt();
}

void FeatureSegCorr::display_t(int t)
{
	if(colorize_flag)
    {
        drawArea()->deleteAllRenderObjects();

        PointSoup * ps = new PointSoup;
		auto points = m1->vertex_coordinates();

		int cout = 0;
		double maxhks = -1;
		double minhks = 99999;
		foreach(Eigen::VectorXd v, HKST)
		{
			if(v.maxCoeff()>maxhks)
				maxhks = v.maxCoeff();
			if(v.minCoeff()<minhks)
				minhks = v.minCoeff();
		}
        foreach(Vertex v, mesh()->vertices())
		{
            ps->addPoint( points[v], qtJetColorMap(HKST[t][cout],minhks,maxhks) );
			cout++;
		}
        drawArea()->addRenderObject(ps);
    }
	else if(t==5)
	{
		drawArea()->deleteAllRenderObjects();

        PointSoup * ps = new PointSoup;
		auto points = m1->vertex_coordinates();

		int cout = 0;
		double maxhks = volume.maxCoeff();
		double minhks = volume.minCoeff();
        foreach(Vertex v, mesh()->vertices())
		{
            ps->addPoint( points[v], qtJetColorMap(volume[cout],minhks,maxhks) );
			cout++;
		}
        drawArea()->addRenderObject(ps);
	}
	else
	{
		drawArea()->deleteAllRenderObjects();

        PointSoup * ps = new PointSoup;
		auto points = m1->vertex_coordinates();

		Eigen::VectorXd diaglp = Eigen::VectorXd::Zero(Lp.cols());
		for(int i = 0;i<Lp.cols();i++)
			diaglp[i] = Lp(i,i);
		int cout = 0;
		double maxhks = -diaglp.minCoeff();
		double minhks = -diaglp.maxCoeff();
        foreach(Vertex v, mesh()->vertices())
		{
            ps->addPoint( points[v], qtJetColorMap(-diaglp[cout],minhks,maxhks) );
			cout++;
		}
        drawArea()->addRenderObject(ps);
	}
	/// forcefully redraw
	{
		drawArea()->updateGL();
		QApplication::processEvents();
	}
}

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
void writeToCSVfile(QString name, Eigen::MatrixXd matrix)
{
	std::ofstream file(name.toStdString().c_str());
	file << matrix.format(CSVFormat);
	file.close();
}

void writeToCSVfile(QString name, Eigen::VectorXd matrix)
{
	std::ofstream file(name.toStdString().c_str());
	file << matrix.format(CSVFormat);
	file.close();
}

bool readFromCSVfile(QString name, Eigen::VectorXd &matrix)
{
	std::ifstream file(name.toStdString().c_str());
	if(!file)
		return false;
	std::string value,line;
	QVector<double> data;
	while(file.good())
	{
		while(std::getline(file,line))
		{
			std::istringstream stream(line);
			while(std::getline(stream,value,','))
			{
				QString qstr = QString::fromStdString(value);
				data.push_back(qstr.toDouble());
			}
		}
	}
	matrix = Eigen::VectorXd::Zero(data.size());
	for(int i = 0; i < data.size(); i++)
		matrix[i] = data[i];
	file.close();
	return true;
}

bool readFromCSVfile(QString name, Eigen::MatrixXd &matrix)
{
	std::ifstream file(name.toStdString().c_str());
	if(!file)
		return false;
	std::string value,in,line;
	QVector<double> data;
	int rows = 0;
	while(file.good())
	{
		while(std::getline(file,line))
		{
			std::istringstream stream(line);
			while(std::getline(stream,value,','))
			{
				QString qstr = QString::fromStdString(value);
				data.push_back(qstr.toDouble());
			}
			rows++;
		}
	}
	int columns = data.size()/rows;
	matrix = Eigen::MatrixXd::Zero(rows,columns);
	for(int i = 0; i < rows; i++)
		for(int j = 0; j < columns; j++)
		{
			matrix(i,j) = data[i*columns + j];
		}
	file.close();
	return true;
}

void FeatureSegCorr::setrunCalcHKS()
{
	HKST.clear();
	// Initial point cloud data to KDtree
	m1 = SurfaceMesh::safe_cast(document()->selectedModel());
	Eigen::Map<Eigen::Matrix3Xd> X((double *)(m1->vertex_coordinates().data()), 3, m1->n_vertices());
	Eigen::Map<Eigen::Matrix3Xd> dst_normals((double *)(m1->vertex_normals().data()), 3, m1->n_vertices());
	SurfaceMesh::VertexCoordinatesIterator vci(m1);
    NanoKDTree3<Vector3> kdtree(vci.begin(), vci.end());

	///////////////////////////////////////////////// For Obj points cloud file
	QString filename = m1->path;
	if(filename.endsWith(".obj"))
	{
		std::ifstream ifs(filename.toStdString());
		char line[1024];
		int count = 0;
		while(ifs.good())
		{
			ifs.getline(line,1024);
			float nx,ny,nz;
			if(strncmp(line, "vn ", 3) == 0)
			{
				int nread = sscanf(line, "vn %f %f %f", &nx, &ny, &nz);
				dst_normals.col(count) = Eigen::Vector3d(nx,ny,nz);
				count++;
			}
		}
	}
	/////////////////////////////////////////////////

	bool has = m1->has_vertex_normals();
	// Get Average distance
	avg = 0;
	for (int i = 0; i < X.cols(); i++)
		avg += kdtree.closest_dist(X.col(i).data());
	avg = avg/X.cols();
	radius = avg*radius;

	// Do the ball search
	double piece_radius = M_PI/4;
	int piece_layer = 3;
	int piece_num = 2*(M_PI/piece_radius)*(M_PI/piece_radius);
	volume = Eigen::VectorXd::Zero(X.cols());
	Eigen::MatrixXd fx = Eigen::MatrixXd::Zero(X.cols(),X.cols());
	QVector<int> numinball;
	for (int i = 0; i < X.cols(); i++)
	{
		Eigen::Vector3d pith = X.col(i);
		Eigen::Vector3d nith = dst_normals.col(i);
		Eigen::Vector3d vith;
		Eigen::Vector3d with;
		nith.normalize();
/*		Eigen::Vector3d n2ith = Eigen::Vector3d::Zero();
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
		Eigen::Vector3d n3ith = n1ith.cross(n2ith);*/
		//////////////////////////////////////////////////
		std::vector<size_t> pointinball = kdtree.ball_search<Eigen::Vector3d>(pith,radius);
		std::vector<double> pointninball_dist = kdtree.ball_search_dist<Eigen::Vector3d>(pith,radius);
		for (int j = 1; j < pointinball.size(); j++)
		{
			vith = nith.cross((pith - X.col(pointinball[j]))/pointninball_dist[j]);
			with = nith.cross(vith);
			double alph = vith.dot(dst_normals.col(pointinball[j]));
			double fai = nith.dot((pith - X.col(pointinball[j]))/pointninball_dist[j]);
			double cita = atan2(with.dot(dst_normals.col(pointinball[j])),nith.dot(dst_normals.col(pointinball[j])));
			fx(i,pointinball[j]) = abs(alph) + abs(fai) + abs(cita)/M_PI;
		}
		numinball.push_back(pointinball.size()-1);
/*		if(pointinball.size()-1 <= 3)
		{
			volume[i] = 0.000000001;
			continue;
		}
		Eigen::MatrixXd affinity = Eigen::MatrixXd::Zero(pointinball.size()-1,pointinball.size()-1);
		for (int j = 1; j < pointinball.size(); j++)
		{
			for (int k = j + 1; k < pointinball.size(); k++)
			{
				affinity(j-1,k-1) = X.col(pointinball[j]).dot(X.col(pointinball[k]));
			}
			affinity(j-1,j-1) = 0;
		}
		double maxd = affinity.maxCoeff();
		affinity = Eigen::MatrixXd::Ones(pointinball.size()-1,pointinball.size()-1)*maxd - affinity;
		affinity = affinity/maxd;
		for (int j = 1; j < pointinball.size(); j++)
			affinity(j-1,j-1) = 0;
		APCluster ap;
		QVector<QVector<int>> clusters = ap.clustering(affinity);
		int testnum = -1;
		if(clusters.size()>1)
			testnum = clusters[1].size();
//		std::vector<double> pointinball_dist = kdtree.ball_search_dist<Eigen::Vector3d>(pith,radius);
//		Eigen::MatrixXd pieces = Eigen::MatrixXd::Zero(piece_num,piece_layer);
/*		for (int j = 1; j < pointinball.size(); j++)
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
		}*/
		Eigen::Matrix3Xd input_points = Eigen::Matrix3Xd::Zero(3,pointinball.size()-1);
		for (int j = 1; j < pointinball.size(); j++)
			input_points.col(j-1) = X.col(pointinball[j]);
		if(pointinball.size()-1 <= 3)
		{
			volume[i] = 0.000000001;
			continue;
		}
		GetConvexHull getqhull(input_points);
		double vvv = getqhull.getVolume();
		volume[i] = vvv;
	}
/*	double maxfx = fx.maxCoeff();
	for (int i = 0; i < X.cols(); i++)
	{
		Eigen::Vector3d pith = X.col(i);
		std::vector<size_t> pointinball = kdtree.ball_search<Eigen::Vector3d>(pith,radius);
		for (int j = 1; j < pointinball.size(); j++)
		{
			double tmpf = fx(i,pointinball[j]);
			fx(i,pointinball[j]) = maxfx - tmpf;
		}
	}*/
//	double maxfx = 3;
//	fx = Eigen::MatrixXd::Constant(X.cols(),X.cols(),maxfx) - fx;

	// Calculate Laplace Operator
	double ballvolume = (4/3)*M_PI*pow(avg,3);
	double t = radius*radius/4;
	Lp = Eigen::MatrixXd::Zero(X.cols(),X.cols());
	for (int j = 0; j < X.cols(); j++)
	{
		for (int k = j; k < X.cols(); k++)
		{
			if(j==k)
				continue;
			double dist = -(X.col(j) - X.col(k)).squaredNorm();
			double l = (1.0f/(pow(4*M_PI*t,1.5)*t))*exp(dist/(4.0f*t));
			Lp(j,k) = l*(ballvolume/4.0f)*fx(j,k)/numinball[j];
			Lp(k,j) = l*(ballvolume/4.0f)*fx(k,j)/numinball[k];
			//(volume[k]/4.0f)
		}
	}
	for (int i = 0; i < X.cols(); i++)
	{
		double ii = - Lp.row(i).sum();
		Lp(i,i) = ii;
	}
	Eigen::MatrixXd Vlp = Eigen::MatrixXd::Zero(X.cols(),X.cols());
	for (int i = 0; i < X.cols(); i++)
		Vlp(i,i) = 1.0f/volume[i];

	double tmin = radius*radius/4;
	double tmax = radius*radius;
	double stepsize = (log(tmax) - log(tmin)) / time_sq_num;
	for (int i = 0; i < time_sq_num; i++)
	{
		double ti = i;
		ti = log(tmin) + ti*stepsize;
		ti = exp(ti);
		Eigen::VectorXd HKS = Eigen::VectorXd::Zero(X.cols());
		Eigen::MatrixXd Lpti = Eigen::MatrixXd::Zero(X.cols(),X.cols());
		for (int j = 0; j < X.cols(); j++)
		{
			for (int k = j; k < X.cols(); k++)
			{
				if(j==k)
					continue;
				double dist = -(X.col(j) - X.col(k)).squaredNorm();
				double l = (1.0f/(pow(4*M_PI*ti,1.5)*ti))*exp(dist/(4.0f*ti));
				Lpti(j,k) = l*(ballvolume/4.0f)*fx(j,k)/numinball[j];
				Lpti(k,j) = l*(ballvolume/4.0f)*fx(k,j)/numinball[k];
			}
		}
		for (int j = 0; j < X.cols(); j++)
		{
			double ii = Lpti.row(j).sum();
			HKS(j) = ii;
		}
		HKST.push_back(HKS);
	}

/*	Eigen::VectorXd egval;
	Eigen::MatrixXd egvec;
	if(!readFromCSVfile(m1->name+"_val.csv",egval)||!readFromCSVfile(m1->name+"_vec.csv",egvec))
	{
		Eigen::EigenSolver<Eigen::MatrixXd> esL(Lp);
		egval = esL.eigenvalues().col(0).real();
		egvec = esL.eigenvectors().real();
		writeToCSVfile(m1->name+"_val.csv",egval);
		writeToCSVfile(m1->name+"_vec.csv",egvec);
	}
	QVector<QPair<double, int>> eigen_sorted;
	for (int i = 0 ; i < X.cols(); i++)
	  eigen_sorted.push_back(QPair<double, int>(abs(egval(i)), i));
	qSort(eigen_sorted.begin(), eigen_sorted.end());
	for (int i = 0 ; i < X.cols(); i++)
		eigen_sorted[i].first = -eigen_sorted[i].first;

	// Calculate HKS k(x,x)
	int maxEig = qMin<double>(1500.0f,X.cols());
	double tmin = abs(4*log(10) / eigen_sorted[X.cols()-1].first);
	double tmax = abs(4*log(10) / eigen_sorted[1].first);
	double stepsize = (log(tmax) - log(tmin)) / time_sq_num;
	double scale = 0;
	for (int i = 0; i < time_sq_num; i++)
	{
		double t = i;
		t = log(tmin) + t*stepsize;
		t = exp(t);
		Eigen::VectorXd HKS = Eigen::VectorXd::Zero(X.cols());
		for (int j = 0; j < X.cols(); j++)
		{
			double h = 0;
			for (int n = 1; n < maxEig; n++)
			{
				h += exp((abs(eigen_sorted[1].first)-abs(eigen_sorted[n].first))*t)*
					egvec(j,eigen_sorted[n].second)*egvec(j,eigen_sorted[n].second);
			}
			HKS[j] = h;
		}
		scale += volume.dot(HKS);
		HKST.push_back(HKS);
	}
	scale = 1.0f/scale;
	for (int i = 0; i < HKST.size(); i++)
	{
		HKST[i] = HKST[i]*scale;
//		writeToCSVfile(QString::number(i)+".csv",HKST[i]);
	}
	*/
}
void FeatureSegCorr::setrunCalcCFF()
{
	m1 = SurfaceMesh::safe_cast(document()->selectedModel());
	Eigen::Map<Eigen::Matrix3Xd> X((double *)(m1->vertex_coordinates().data()), 3, m1->n_vertices());
	SurfaceMesh::VertexCoordinatesIterator vci(m1);
    NanoKDTree3<Vector3> kdtree(vci.begin(), vci.end());

	Eigen::MatrixXd DistHKS = Eigen::MatrixXd::Constant(X.cols(),X.cols(),999999.9f);
	Eigen::MatrixXd AffinityHKS = Eigen::MatrixXd::Zero(X.cols(),X.cols());
	Eigen::VectorXd Sig = Eigen::VectorXd::Zero(X.cols());
	for(int i = 0; i < X.cols(); i++)
	{
		DistHKS(i,i) = 0;
		Eigen::Vector3d pith = X.col(i);
		std::vector<size_t> pointinball = kdtree.ball_search<Eigen::Vector3d>(pith,avg*rcff);
		for(int j = 1; j < pointinball.size(); j++)
		{
			double d = 0;
			for(int k = 0; k < time_sq_num; k++)
			{
				d += pow(HKST[k][i] - HKST[k][pointinball[j]],2);
			}
			DistHKS(i,pointinball[j]) = d;
			if( j == sigma )
				Sig[i] = d;
		}
	}
	for(int i = 0; i < X.cols(); i++)
	{
		for(int j = i; j < X.cols(); j++)
		{
			if(i==j)
				continue;
			AffinityHKS(i,j) = exp(-DistHKS(i,j)/(Sig[i]*Sig[j]));
		}
	}


}
