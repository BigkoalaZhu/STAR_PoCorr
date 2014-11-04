#include "APCluster.h"

#ifndef MINDOUBLE
#define MINDOUBLE 2.2250e-308
#endif
#ifndef MAXDOUBLE
#define MAXDOUBLE 1.7976e308
#endif

APCluster::APCluster()
{
}

QVector<QVector<int>> APCluster::clustering(Eigen::MatrixXd affinity)
{
	QVector<int> idx = getIdx(affinity);

	QVector<QVector<int>> clusters;
	for (int i=0; i<idx.size(); i++)
	{
		int cIdx = -1;
		for (int j=0; j<clusters.size(); j++)
		{
			if(idx[clusters[j][0]] == idx[i])
			{
				cIdx = j;
			}
		}

		if (cIdx == -1)
		{
			QVector<int> newCluster;
			newCluster << i;
			clusters << newCluster;
		}
		else
		{
			clusters[cIdx] << i;
		}
	}

	return clusters;
}

QVector<double> APCluster::getPreference( Eigen::MatrixXd affinity )
{
	QVector<double> preference;	

	// 1. Initially, set the preference to be the median affinity for each node
	double maxPreference = -100;
	for (int ir=0; ir<affinity.rows(); ir++)
	{
		QVector<double> v;
		for (int ic=0; ic<affinity.cols(); ic++)
		{
			if (ir != ic)
			{
				v << affinity(ir,ic);
			}
		}
		qSort(v);

		if (v.size()%2 == 0)
		{
			preference << (v[v.size()/2] + v[v.size()/2-1])/2;
		}
		else
		{
			preference << v[v.size()/2];
		}

		if (preference.last() > maxPreference)
		{
			maxPreference = preference.last();
		}
	}

	// 2. For those nodes that doesn't have any node with high affinity, set high preference so that it can form a cluster by itself
	QVector<double> maxAffinity;
	for (int ir=0; ir<affinity.rows(); ir++)
	{
		maxAffinity << affinity.row(ir).maxCoeff();
	}
	
	// testing: set a threshold for now
//	for (int i=0; i<maxAffinity.size(); i++) 
//	{
//		if (maxAffinity[i] < 0.7)
//		{
//			preference[i] = 1.0;
//		}
//	}


	return preference;
}

QVector<int> APCluster::getIdx( Eigen::MatrixXd affinity )
{
	QVector<int> index;

	int dn, it, conv, decit, maxits, convits, m, n, j, i1;
	unsigned long *i, *k,  **dec, *decsum, *idx, K;
	double tmp, *s, *a, *r, *mx1, *mx2, *srp, netsim, dpsim, expref, lam;

	if (MINDOUBLE==0.0) 
	{
		qDebug("There are numerical precision problems on this architecture.  Please recompile after adjusting MIN_DOUBLE and MAX_DOUBLE");
	}

	/* Parse command line */
	lam=0.5; maxits=500; convits=50;

	/* Find out how many data points and similarities there are */
	n = affinity.rows();
	m = affinity.rows() * affinity.cols() - n;

	if (m == 0)
	{
		index.push_back(0);
		return index;
	}

	/* Allocate memory for similarities, preferences, messages, etc */
	i=(unsigned long *)calloc(m+n,sizeof(unsigned long));
	k=(unsigned long *)calloc(m+n,sizeof(unsigned long));
	s=(double *)calloc(m+n,sizeof(double));
	a=(double *)calloc(m+n,sizeof(double));
	r=(double *)calloc(m+n,sizeof(double));
	mx1=(double *)calloc(n,sizeof(double));
	mx2=(double *)calloc(n,sizeof(double));
	srp=(double *)calloc(n,sizeof(double));
	dec=(unsigned long **)calloc(convits,sizeof(unsigned long *));
	for(j=0;j<convits;j++)
		dec[j]=(unsigned long *)calloc(n,sizeof(unsigned long));
	decsum=(unsigned long *)calloc(n,sizeof(unsigned long));
	idx=(unsigned long *)calloc(n,sizeof(unsigned long));

	/* Read similarities and preferences */
	for (int ir=0; ir<affinity.rows(); ir++)
	{
		for (int ic=0; ic<affinity.cols(); ic++)
		{
			if (ir != ic)
			{
				s[ir*affinity.cols()+ic] = affinity(ir,ic);
				i[ir*affinity.cols()+ic] = ir;
				k[ir*affinity.cols()+ic] = ic;
			}
		}
	}

	/* Include a tiny amount of noise in similarities to avoid degeneracies */
	for(j=0;j<m;j++) s[j]=s[j]+(1e-16*s[j]+MINDOUBLE*100)*(rand()/((double)RAND_MAX+1));

	QVector<double> preference = getPreference(affinity);
	for(j=0;j<n;j++){
		i[m+j]=j; 
		k[m+j]=j; 
		s[m+j] = preference[j];
	}

	m=m+n;

	/* Initialize availabilities to 0 and run affinity propagation */
	for(j=0;j<m;j++) a[j]=0.0;
	for(j=0;j<convits;j++) for(i1=0;i1<n;i1++) dec[j][i1]=0;
	for(j=0;j<n;j++) decsum[j]=0;
	dn=0; it=0; decit=convits;
	while(dn==0){
		it++; /* Increase iteration index */

		/* Compute responsibilities */
		for(j=0;j<n;j++){ mx1[j]=-MAXDOUBLE; mx2[j]=-MAXDOUBLE; }
		for(j=0;j<m;j++){
			tmp=a[j]+s[j];
			if(tmp>mx1[i[j]]){
				mx2[i[j]]=mx1[i[j]];
				mx1[i[j]]=tmp;
			} else if(tmp>mx2[i[j]]) mx2[i[j]]=tmp;
		}
		for(j=0;j<m;j++){
			tmp=a[j]+s[j];
			if(tmp==mx1[i[j]]) r[j]=lam*r[j]+(1-lam)*(s[j]-mx2[i[j]]);
			else r[j]=lam*r[j]+(1-lam)*(s[j]-mx1[i[j]]);
		}

		/* Compute availabilities */
		for(j=0;j<n;j++) srp[j]=0.0;
		for(j=0;j<m-n;j++) if(r[j]>0.0) srp[k[j]]=srp[k[j]]+r[j];
		for(j=m-n;j<m;j++) srp[k[j]]=srp[k[j]]+r[j];
		for(j=0;j<m-n;j++){
			if(r[j]>0.0) tmp=srp[k[j]]-r[j]; else tmp=srp[k[j]];
			if(tmp<0.0) a[j]=lam*a[j]+(1-lam)*tmp; else a[j]=lam*a[j];
		}
		for(j=m-n;j<m;j++) a[j]=lam*a[j]+(1-lam)*(srp[k[j]]-r[j]);

		/* Identify exemplars and check to see if finished */
		decit++; if(decit>=convits) decit=0;
		for(j=0;j<n;j++) decsum[j]=decsum[j]-dec[decit][j];
		for(j=0;j<n;j++)
			if(a[m-n+j]+r[m-n+j]>0.0) dec[decit][j]=1; else dec[decit][j]=0;
		K=0; for(j=0;j<n;j++) K=K+dec[decit][j];
		for(j=0;j<n;j++) decsum[j]=decsum[j]+dec[decit][j];
		if((it>=convits)||(it>=maxits)){
			/* Check convergence */
			conv=1; for(j=0;j<n;j++) if((decsum[j]!=0)&&(decsum[j]!=convits)) conv=0;
			/* Check to see if done */
			if(((conv==1)&&(K>0))||(it==maxits)) dn=1;
		}
	}
	/* If clusters were identified, find the assignments and output them */
	if(K>0){
		for(j=0;j<m;j++)
			if(dec[decit][k[j]]==1) a[j]=0.0; else a[j]=-MAXDOUBLE;
		for(j=0;j<n;j++) mx1[j]=-MAXDOUBLE;
		for(j=0;j<m;j++){
			tmp=a[j]+s[j];
			if(tmp>mx1[i[j]]){
				mx1[i[j]]=tmp;
				idx[i[j]]=k[j];
			}
		}
		for(j=0;j<n;j++) if(dec[decit][j]) idx[j]=j;
		for(j=0;j<n;j++) srp[j]=0.0;
		for(j=0;j<m;j++) if(idx[i[j]]==idx[k[j]]) srp[k[j]]=srp[k[j]]+s[j];
		for(j=0;j<n;j++) mx1[j]=-MAXDOUBLE;
		for(j=0;j<n;j++) if(srp[j]>mx1[idx[j]]) mx1[idx[j]]=srp[j];
		for(j=0;j<n;j++)
			if(srp[j]==mx1[idx[j]]) dec[decit][j]=1; else dec[decit][j]=0;
		for(j=0;j<m;j++)
			if(dec[decit][k[j]]==1) a[j]=0.0; else a[j]=-MAXDOUBLE;
		for(j=0;j<n;j++) mx1[j]=-MAXDOUBLE;
		for(j=0;j<m;j++){
			tmp=a[j]+s[j];
			if(tmp>mx1[i[j]]){
				mx1[i[j]]=tmp;
				idx[i[j]]=k[j];
			}
		}
		for(j=0;j<n;j++) if(dec[decit][j]) idx[j]=j;
		
		index.clear();
		for(j=0;j<n;j++) 
		{
			index.push_back(idx[j]+1);  /* success: print out the indices */
		}

		dpsim=0.0; expref=0.0;
		for(j=0;j<m;j++){
			if(idx[i[j]]==k[j]){
				if(i[j]==k[j]) expref=expref+s[j];
				else dpsim=dpsim+s[j];
			}
		}
		netsim=dpsim+expref;
		qDebug("\nNumber of identified clusters: %d\n",K);
		qDebug("Fitness (net similarity): %f\n",netsim);
		qDebug("  Similarities of data points to exemplars: %f\n",dpsim);
		qDebug("  Preferences of selected exemplars: %f\n",expref);
		qDebug("Number of iterations: %d\n\n",it);
	} 
	else 
	{
		qDebug("\nDid not identify any clusters\n");
	}
	if(conv==0)
	{
		qDebug("\n*** Warning: Algorithm did not converge. Consider increasing\n");
		qDebug("    maxits to enable more iterations. It may also be necessary\n");
		qDebug("    to increase damping (increase dampfact).\n\n");
	}

	return index;
}

