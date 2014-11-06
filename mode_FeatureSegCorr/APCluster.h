#pragma once

#include "Eigen/Dense"
#include <QVector>

class APCluster
{
public:
	APCluster();
	QVector<QVector<int>> clustering(Eigen::MatrixXd affinity, QVector<double> p);

private:
	QVector<int> getIdx(Eigen::MatrixXd affinity);
	QVector<double> getPreference( Eigen::MatrixXd affinity );
	QVector<double> pre;
};