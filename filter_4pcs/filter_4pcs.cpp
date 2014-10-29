#include "filter_4pcs.h"

#include <QElapsedTimer>
#include <fstream>
#include <cfloat>
#include <limits>
#include <iomanip>
#include "4pcs_1.3/4pcs.h"

void filter_4pcs::initParameters(RichParameterSet* pars){
    drawArea()->deleteAllRenderObjects();
    
    /// Avoids inserting selection 
    QStringList layers;
    foreach(StarlabModel* model, document()->models())
        if(SurfaceMesh::isA(model) && model!=mesh())
            layers << model->name;
	
    pars->addParam(new RichFloat("delta", 5.0, "Delta"));
	pars->addParam(new RichInt("n_points", 200, "Number of sampled points in both files"));
    pars->addParam(new RichFloat("overlap", 0.2, "Overlap"));
    pars->addParam(new RichFloat("thr", 1.0, "Threshold of the computed overlap for termination"));
    pars->addParam(new RichFloat("max_color", 1e9, "Maximum norm of RGB values between corresponded points"));
    pars->addParam(new RichFloat("norm_diff", 10.0, "Maximum angle (degrees) between corresponded normals"));
    pars->addParam(new RichStringSet("layer", layers, "Align with this mesh"));
}

void filter_4pcs::applyFilter(RichParameterSet* pars) {
    std::cout << std::setprecision(std::numeric_limits<double>::digits10);
    QElapsedTimer timer;

    /// Fetch parameters
    double delta = pars->getFloat("delta");
	int n_points = pars->getInt("n_points");
    double overlap = pars->getFloat("overlap");
    double thr = pars->getFloat("thr");
	double max_color = pars->getFloat("max_color");
	double norm_diff = pars->getFloat("norm_diff");
    QString layer = pars->getString("layer");
    
    //////////////////////////////////////////////////
    Model *m1 = SurfaceMesh::safe_cast(document()->selectedModel());
    Model *m2 = SurfaceMesh::safe_cast(document()->getModel(layer));
    m1->color = QColor(186, 172, 96);
    m2->color = QColor(86, 117, 147);
    
    //////////////////////////////////////////////////
    Eigen::Map<Eigen::Matrix3Xd> X((double *)(m1->vertex_coordinates().data()), 3, m1->n_vertices());
	Eigen::Map<Eigen::Matrix3Xd> X_normals((double *)(m1->vertex_normals().data()), 3, m1->n_vertices());
    Eigen::Map<Eigen::Matrix3Xd> dst((double *)(m2->vertex_coordinates().data()), 3, m2->n_vertices());
    Eigen::Map<Eigen::Matrix3Xd> dst_normals((double *)(m2->vertex_normals().data()), 3, m2->n_vertices());
	//////////////////////////////////////////////////
	std::vector<match_4pcs::Point3D> Q;
	std::vector<match_4pcs::Point3D> P;
	for(int i = 0; i < X.cols(); i++)
	{
		match_4pcs::Point3D q(X(0,i),X(1,i),X(2,i));
		q.set_normal(cv::Point3d(X_normals(0,i),X_normals(1,i),X_normals(2,i)));
		Q.push_back(q);
	}
	for(int i = 0; i < dst.cols(); i++)
	{
		match_4pcs::Point3D p(dst(0,i),dst(1,i),dst(2,i));
		p.set_normal(cv::Point3d(dst_normals(0,i),dst_normals(1,i),dst_normals(2,i)));
		P.push_back(p);
	}
	//////////////////////////////////////////////////
	// Our matcher.
	match_4pcs::Match4PCSOptions options;

	// Set parameters.
	cv::Mat mat = cv::Mat::eye(4,4, CV_64F);
	options.overlap_estimation = overlap;
	options.max_distance = delta;
	options.sample_size = n_points;
	options.max_normal_difference = norm_diff;
	options.max_color_distance = max_color;

	match_4pcs::Match4PCS matcher(options);
	float score = matcher.ComputeTransformation(P, &Q, &mat);
	for(int i = 0; i < X.cols(); i++)
	{
		X.col(i) = Eigen::Vector3d(Q[i].x,Q[i].y,Q[i].z);
	}

	/// forcefully redraw
	{
		drawArea()->updateGL();
		QApplication::processEvents();
	}
}
