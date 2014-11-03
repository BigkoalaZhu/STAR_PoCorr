#include "GetConvexHull.h"


GetConvexHull::GetConvexHull(Eigen::Matrix3Xd points)
{
	QVector<SurfaceMesh::Vec3d> Vps; 
	for(int i = 0; i < points.cols(); i++)
		Vps.push_back(points.col(i));
	qhull = new orgQhull::Qhull("", 3, (int)Vps.size(), Vps[0].data(), "s FA");
	volume = qhull->volume();
}


GetConvexHull::~GetConvexHull(void)
{
	delete qhull;
}

double GetConvexHull::getVolume()
{
	return volume;
}
