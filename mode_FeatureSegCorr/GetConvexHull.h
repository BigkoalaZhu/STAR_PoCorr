#pragma once
///////////////////////////////////
#include "Qhull.h"
#include "SurfaceMeshHelper.h"
//////////////////////////////////

class GetConvexHull
{
public:
	GetConvexHull(Eigen::Matrix3Xd points);
	double getVolume();
	~GetConvexHull(void);
private:
	double volume;
	orgQhull::Qhull *qhull;
};

