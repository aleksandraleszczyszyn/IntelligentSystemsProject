#include <math.h>
#include <limits>
#include <stdlib.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "kmeans.h"
#include "FeatureMetadata.h"
#include "Cluster.h"
#include "DefaultPointRepresentation.h"
#include "BssBag.h"
#include "WssBag.h"

#include <tr1/unordered_set>

using std::tr1::unordered_set;

using namespace std;
using namespace pcl;

/*
KMeans::KMeans(int maxIters, int maxCheckIters, float splitThresh, float mergeThresh, float bssThresh,
               float wtssThresh, float wtssImproveThresh)
    :   lastWssClusterSplit(0), maxIters(maxIters), maxCheckIters(maxCheckIters), splitThresh(splitThresh), 
		mergeThresh(mergeThresh), bssThresh(bssThresh), wtssThresh(wtssThresh), wtssImproveThresh(wtssImproveThresh){
*/
KMeans::KMeans(int maxIters)
    : maxIters(maxIters) {

	maxCheckIters=1;
	wssThreshForSplit = 0.12;
	minSplitClusterSize = 50;
	degenerateClusterSize = 20;
	wssThreshForMerge = 0.07;
	maxSplitClusterSize = 500;
	bssThreshForMerge = 0.14;
}

void KMeans::init(Memory &mem){
    memory = &mem;
}

//NEW//
void KMeans::update()
{
	checkValidity();
	/*while(!checkClustersValidity()) {
		//fit();
	}*/
}

/*
void KMeans::fit2() {

	cout << "fit2" << endl;

    // called if the quality of the cluster should be improved (via a merge or a split)
    bool assignChanged;

    // false means the centroids should only be updated after all the assignments have been done
    bool upCentroids = false;

    // no assigment cluster is specified
    int assgnCluster = -1;
    
    // the number of neighbours to take into account when applying the nearestKSearch step
    int k = 1;

    // variables to hold cluster previous and new assignments
    int pCluster;
    int nCluster;

    unordered_map<int, FeatureMetadata> & featsM = memory->getFeatures();
    
    for(int iter = 1; iter <= maxIters; ++iter) {
       
        assignChanged = false;

        // the centroids of the clusters
        PointCloud<Histogram<153> >::Ptr centroids (new PointCloud<Histogram<153> >);

        // maps the cluster centroids in the 'centroids' vector, to the respective cluster identifiers in 'mapper'
        vector<int> mapper;
    
        // get clusters centroids
        getCentroids(centroids, mapper);
        
        // iterate over all points and assign features (points) to clusters
        for(unordered_map<int, FeatureMetadata>::iterator featM=featsM.begin(); featM != featsM.end(); ++featM) {

	    if(featM->second.getCluster() == -1)
	        continue;

            bool isNewAssign = assignToClusters(centroids, mapper,
                    featM->second, featM->first, k, upCentroids, assgnCluster, pCluster, nCluster);
            
            assignChanged = assignChanged || isNewAssign;
        }

        // if there were no changes in cluster assignments exit (KMeans has converged)
        if(!assignChanged) {
            return;
        }
        
        // update the clusters centroids
        recomputeCentroids();
    }
}
*/

void KMeans::assign(vector<int> &featsIdx, int assignCluster) {  

    // caled every time new features are added to memory

	// assignCluster defaults to '-1' meaning that the features should be assigned to
	// the closest cluster as opposed to being assigned to a specific cluster (specified in that case by the value of assignCluster)
    
	// perform only one iteration on assignments
	// update and recompute centroids on each new feature assignment
    assignFeatures(featsIdx, assignCluster);
    
    // after cluster assigment check cluster validity
    // decide if a merge or a split is necessary in order to improve cluster quality
    // check if no improvement was achieved to end the check

	checkValidity();
    /*while(!checkClustersValidity()) {
        //fit();
    }*/
}

void KMeans::assignFeatures(vector<int> &featsIdx, int &assignCluster, bool upCentroids, int maxIts) {  

    // caled every time new features are added to memory
	// called if the quality of the cluster should be improved (via a merge or a split)
    
    // upCentroids defaults to true meaning that the centroids should be updated after each new assignment
    
    // the number of neighbours to take into account when applying the nearestKSearch step
    int k = 1;

    // variables to hold cluster previous and new assignments
    int pCluster;
    int nCluster;
    
    // the centroids of the clusters
    PointCloud<Histogram<153> >::Ptr centroids (new PointCloud<Histogram<153> >);

    // mapper for the centroids vector indexes position  -> to the cluster identifiers
    vector<int> mapper;
    
	// reverse of mapper, from the cluster identifiers -> to the centroids vector indexes position
	unordered_map<int, int> revMapper;
		
    // get clusters centroids
    getCentroids(centroids, mapper, revMapper);
        
	bool assignChanged;
	
	for(int iter = 1; iter <= maxIts; ++iter) {
        //cout << "assignment iteration : " << iter << " of : " << maxIts << endl;
        
        assignChanged = false;
		
		// assign all features in the vector to clusters
		for(vector<int>::iterator featIdx = featsIdx.begin(); featIdx != featsIdx.end(); ++featIdx){
			FeatureMetadata &featMetadata = memory->getFeatureMetadata(*featIdx);
			 bool isNewAssign = assignToClusters(
				 	centroids, mapper, revMapper, featMetadata, *featIdx, k, upCentroids, assignCluster, pCluster, nCluster);
			
			assignChanged = assignChanged || isNewAssign;
		}
		
		// if there were no changes in cluster assignments exit (KMeans has converged)
		// if only 1 iteration is specified this means we are assigninig the fetures for the first time
		// therefore no changes in assignment will be detected. This will also happen with split/merge
		// because these methods unassign the features.
		// So in the case of 1 iteration specified the for loop will exit naturrally. In case of more then 1
		// iteration specified this prevents the loop from exiting upont the first iteration since in this 
		// first iteration only unassigned features will be processed as the algorithm stands
		if(!assignChanged && iter > 1) {
			return;
		}
	}
	
	if (!upCentroids) {
		// if the update of the clusters was set to only be performed after all the
		// features have been assigned to, perform the update now
		recomputeCentroids();
	}
}

bool KMeans::assignToClusters(PointCloud<Histogram<153> >::Ptr &centroids, vector<int> &mapper,
                              unordered_map<int, int> &revMapper, FeatureMetadata &featMetadata, const int &featIdx, int k,
                              bool &upCentroids, int &assignCluster, int &pCluster, int &nCluster) {

    bool assignChanged;
    
    vector<int> kIndices(k);
    vector<float> kSqrDistances(k);

    // set the default assignment cluster
    nCluster = assignCluster;
    
    // take the weight in consideration if weight is diferent from 1
    Histogram<153> feat = featMetadata.getFeature();
    if(featMetadata.getGlobalWeight() != 1.0f) {
        for(int i = 0; i < 153; ++i) {
            feat.histogram[i] = feat.histogram[i] * featMetadata.getGlobalWeight();
        }
    }
    
    // retain the old cluster assignment
    pCluster = featMetadata.getCluster();

    // if no assigment cluster was specified assign the features according to the nearest cluster
    if (assignCluster == -1) {
        // find the k neares clusters thru the centroids
        findNearestPoint(centroids, feat, kIndices, kSqrDistances, k);
        
        // get new cluster identifier from the mapper (centroid vector index to cluster identifier)
        nCluster = mapper.at(kIndices.at(0));

        // set distance from feature (point) to centroid - possibly to remove
        //featMetadata.setDistanceToCentroid(sqrt(kSqrDistances.at(0)));
    }
    
    // assign the feature (point) to the nearest found cluster
    assignChanged = featMetadata.assignToCluster(nCluster);

    // if feature changed assigned cluster
    if(assignChanged) {
		// synchronize cluster assignment for the feature (remove from old assignment and add to new assignment)
        memory->syncClusterAssignment(pCluster, nCluster, featIdx);

        // update the cluster centroids (this update is done after each assignment)
        // if cluster assignment changed, the centroids of both the previous and new
        // cluster assignment must be updated
        if(upCentroids) {

            // update cluster's running totals
            memory->updateCentroids(pCluster, nCluster, featIdx);

            // update the centroids pointcloud in memory
            // this structure is built upon the cluster centroids
            // and is needed in this format for the pcl nearestKSearch method
            // in this case we are only considering the assign case so
            // assuming that only new features will be added at the closest cluster = kIndices.at(0) (position in the centroids vector)
            // if the fit scenario is also considered then the previous cluster's centroid should also be updated

            // new assigned cluster update for next assignment
            centroids->at(kIndices.at(0)) = memory->getClusterCentroid(nCluster);

            // previous assigned cluster update
			if (pCluster >= 0) {
				// if feature was assigned to another cluster, then the feature was removed from that cluster
				// and therefore the centroid of that cluster should also be updated in the in-memory centroids structure
            	centroids->at(revMapper[pCluster]) = memory->getClusterCentroid(pCluster);
			}
        }
    }
    
    return assignChanged;
}

void KMeans::getCentroids(
	PointCloud<Histogram<153> >::Ptr &centroids, vector<int> &mapper, unordered_map<int, int> &revMapper){
    
    memory->getCentroids(centroids, mapper, revMapper);
    if(centroids->empty()){
        // if there are no centroids yet (i.e. no clusters)
        // create a cluster and set it's centroid as any random point
        // already in memory (e.g. the first point)
        memory->addCluster(memory->getFeature(0));
        // with a cluster defined get the centroids again
        memory->getCentroids(centroids, mapper, revMapper);
    }
}


/*
//void KMeans::fit(vector<int> &featsIdx){  
void KMeans::fit() {

    // called if the quality of the cluster should be improved (via a merge or a split)
    bool assignChanged;

    // false means the centroids should only be updated after all the assignments have been done
    bool upCentroids = false;

    // no assigment cluster is specified
    int assgnCluster = -1;
    
    // the number of neighbours to take into account when applying the nearestKSearch step
    int k = 1;

    // variables to hold cluster previous and new assignments
    int pCluster;
    int nCluster;

    unordered_map<int, FeatureMetadata> & featsM = memory->getFeatures();
    
    for(int iter = 1; iter <= maxIters; ++iter) {
        //cout << "KMeans iteration " << iter << endl;
        
        assignChanged = false;

        // the centroids of the clusters
        PointCloud<Histogram<153> >::Ptr centroids (new PointCloud<Histogram<153> >);

        // maps the cluster centroids in the 'centroids' vector, to the respective cluster identifiers in 'mapper'
        vector<int> mapper;
    
        // get clusters centroids
        getCentroids(centroids, mapper);
        
        // iterate over all points and assign features (points) to clusters
        for(unordered_map<int, FeatureMetadata>::iterator featM=featsM.begin(); featM != featsM.end(); ++featM) {
            bool isNewAssign = assignToClusters(centroids, mapper,
                    featM->second, featM->first, k, upCentroids, assgnCluster, pCluster, nCluster);
            
            assignChanged = assignChanged || isNewAssign;
        }

        // if there were no changes in cluster assignments exit (KMeans has converged)
        if(!assignChanged) {
            return;
        }
        
        // update the clusters centroids
        recomputeCentroids();
    }
}
*/

void KMeans::recomputeCentroids() {

    // recomputes the centroids using all the points of the cluster
    
    unordered_map<int, Cluster> &clusters = memory->getClusters();
    for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
        // for each cluster
        Cluster &cluster = clt->second;
        unordered_set<int> &clusterPointsIdx = cluster.getPoints();

        // reset the running totals for the cluster
        cluster.resetClusterTotals();

        // iterate over all features (points) assigned to the cluster
        for(unordered_set<int>::iterator ptIdx=clusterPointsIdx.begin(); ptIdx != clusterPointsIdx.end(); ++ptIdx) {
            FeatureMetadata &featMetadata = memory->getFeatureMetadata(*ptIdx);
            // keep total feature sum (i.e. the cluster sumFeatures)
            cluster.addToSumFeatures(memory->getFeature(*ptIdx), featMetadata.getGlobalWeight());

            // keep the sum of all weights
            cluster.addToSumWeight(featMetadata.getGlobalWeight());
        }
        
        // recompute the cluster centroid base on the running totals
        cluster.recomputeCentroid();
    }
}

void KMeans::findNearestPoint(const PointCloud<Histogram<153> >::Ptr &points,
            const Histogram<153> &point, vector<int> &kIndices, vector<float> &kSqrDistances, int k) {

    KdTreeFLANN<Histogram<153> > kdtree;
    kdtree.setInputCloud(points);
    kdtree.nearestKSearch(point, k, kIndices, kSqrDistances);
}

void KMeans::checkValidity() {
	
	for(int iter = 1; iter <= maxCheckIters; ++iter) {
		//cout << "Validity check iteration: " << iter << endl;
		
		// keep track of the modified clusters
		vector<int> touchedClusters;
		
		//bool validCheck = true;
		//validCheck = checkClustersValidity(touchedClusters) ;
		checkClustersValidity(touchedClusters) ;
		
		// remove any resulting empty cluster
		cleanEmptyClusters(touchedClusters);
		
		if(touchedClusters.empty()) {
			break;
		}
	}
}
	
/*
bool KMeans::checkClustersValidity() {

	bool validCheck = true;

	float wssClusterSplitThreshold = 0.125;
	//float wssSplitThreshold = 0.1;
	float wssMergeThreshold = 0.075;
	float bssMergeThreshold = 0.18;
	int minSplitClusterSize = 10;

	// holds the wss computed for each cluster
	vector<pair<int, WssBag> > clustersWss;

	// wss is the total WSS computed over all clusters
	// WSS stands for Within cluster Sum of Squares
	float avgWss = memory->wss(clustersWss);
	// check if any cluster exceeds the threshold imposed in the cluster WSS
	if(clustersWss.at(0).second.getNormalizedWss() > wssClusterSplitThreshold &&
	  memory->getClusterSize(clustersWss.at(0).first) > minSplitClusterSize) {
	//if((avgWss / clustersWss.at(0).second.getNormalizedWss()) < 0.5) {
	//if(clustersWss.at(0).second.getNormalizedWss() > wssClusterSplitThreshold && (clustersWss.at(0).second.getNormalizedWss() - lastWssClusterSplit) >= 0.01) {
	//if(wss > wssSplitThreshold && abs(clustersWss.at(0).second.getNormalizedWss() - lastWssClusterSplit) >= 0.025) {
	//if(clustersWss.at(0).second.getNormalizedWss() > wssClusterSplitThreshold && wss > wssSplitThreshold) {

		// signal a split should follow this operation
		validCheck = false;

		
		cout 	<< "spliting cluster with wss: " 
			<< clustersWss.at(0).second.getNormalizedWss() << endl;

		// split cluster
		splitCluster(clustersWss.at(0).first);
		lastWssClusterSplit = clustersWss.at(0).second.getNormalizedWss();
	}
	else {
		// check if a merge should be attempted
		if (clustersWss.size() > 1) {
			// clusters in the last positions of the vector clustersWss
			// have the lowest WSS measures
			for (int i = clustersWss.size() - 1; i > 0; --i) {
				//cout << "cluster 1 wss " << clustersWss.at(i).second.getNormalizedWss() << endl;
				// if the first cluster WSS is below the merging threshold
				if(clustersWss.at(i).second.getNormalizedWss() <= wssMergeThreshold) {
					// try to find a second cluster below the WSS merging threshold
					for (int j = i - 1; j >= 0; --j) {
						//cout << "cluster 2 wss " << clustersWss.at(j).second.getNormalizedWss() << endl;
						if(clustersWss.at(j).second.getNormalizedWss() <= wssMergeThreshold) {
							// compute BSS between both clusters and compare with threshold
							float bssClusters = memory->bssBetwenClusters(clustersWss.at(i).first, clustersWss.at(j).first);
							

							cout << "bss cluster 1 : " << clustersWss.at(i).first << " and cluster 2 : "
								<< clustersWss.at(j).first << " , is : " << bssClusters << endl;


							if(bssClusters < bssMergeThreshold) {

								cout << "merging cluster " << clustersWss.at(i).first << " with wss: "
										<< clustersWss.at(i).second.getNormalizedWss() << " with cluster " << clustersWss.at(j).first
										<< " with wss: " << clustersWss.at(j).second.getNormalizedWss() << endl;


								// merge clusters
								mergeClusters(clustersWss.at(i).first, clustersWss.at(j).first);
								return false;
							}
						}
						else {
							break;
						}
					}
				}
				else {
					break;
				}
			}
		}
	}
		
    return validCheck;
}
*/

void KMeans::checkClustersValidity(vector<int> &touchedClusters) {

	// holds the wss computed for each cluster
	vector<pair<int, WssBag> > clustersWss;
	
	// wss is the total WSS computed over all clusters
	// WSS stands for Within cluster Sum of Squares
	float avgWss = memory->wss(clustersWss);
	
	int currInd = 0;
	
	// chack all clusters
	for(vector<pair<int, WssBag> >::iterator wss=clustersWss.begin(); wss != clustersWss.end(); ++wss) {
		
		// set clusters as operatedOn after split or merge operatios
		// so that, and more in particular in the merge case, clusters
		// that have been removed in previous operations 
		// are not further operated on
		
		if (checkDegenerate(wss->second)) {
			//cout 	<< "removing degenerate cluster: " << wss->first << endl;
			cleanDegenerateCluster(wss->first);
		}
		else if (checkSplitConditions(wss->second)) {
			//cout 	<< "spliting cluster with wss: " << wss->second.getNormalizedWss() << endl;
			
			// split cluster
			splitCluster(wss->first, touchedClusters);
			wss->second.setOperatedOn();
		}
		else if (checkMergeConditions(wss->second)) {

				// try to find a second cluster suitable for merging with this cluster
				// mCluster holds the index of the cluster for merging.
				// in case no suitable cluster can be found -1 is returned
				int mCluster = findClusterToMerge(clustersWss, currInd);
				
				if(mCluster > -1) {
					clustersWss.at(mCluster).second.setOperatedOn();
					wss->second.setOperatedOn();
					mergeClusters(wss->first, clustersWss.at(mCluster).first, touchedClusters);
				}
		}
		
		++currInd;
	}
}

bool KMeans::checkDegenerate(WssBag &clusterMeta) {
	
	// check if the size of the cluster obeys the minimum cluster size for degenerate
	bool degenerateSize = clusterMeta.getCount() <= degenerateClusterSize;

	bool notOperatedOn = !clusterMeta.asBeenOperatedOn();

	return notOperatedOn && degenerateSize;
}
	
bool KMeans::checkSplitConditions(WssBag &clusterMeta) {
	
	// check if the size of the cluster obeys the minimum cluster size for splitting operations
	// and that the cluster wss exceeds the specified theshold
	bool wssMinSize = clusterMeta.getCount() > minSplitClusterSize && clusterMeta.getNormalizedWss() > wssThreshForSplit
		&& clusterMeta.getMaxDistance() > 2 * wssThreshForSplit;
	
	// check if the size of the cluster as reached the maximum allowed	and that the cluster wss exceeds the specified theshold
	//bool wssMaxSize = clusterMeta.getCount() > maxSplitClusterSize && (clusterMeta.getNormalizedWss() > wssThreshForSplit
	//																   || clusterMeta.getMaxDistance() > 2 * wssThreshForSplit);
	//bool wssMaxSize = clusterMeta.getCount() > maxSplitClusterSize && clusterMeta.getNormalizedWss() > wssThreshForSplit;	
	
	// cluster has not been previously operated on e.g a split operation
	bool notOperatedOn = !clusterMeta.asBeenOperatedOn();
	
	//return notOperatedOn && (wssMinSize || wssMaxSize);
	return notOperatedOn && wssMinSize;
}

bool KMeans::checkMergeConditions(WssBag &clusterMeta) {
	
	// check if the cluster wss is below the threshold to be considered for merging
	bool wssMin = clusterMeta.getNormalizedWss() <= wssThreshForMerge;
	
	bool minClusterSize = clusterMeta.getCount() > 0;
	
	// cluster has not been previously operated on e.g a split operation
	bool notOperatedOn = !clusterMeta.asBeenOperatedOn();
	
	return minClusterSize && wssMin && notOperatedOn;
}

int KMeans::findClusterToMerge(vector<pair<int, WssBag> > &clustersWss, int &currInd) {
	
	// start searching for suitable clusters immediately after the current position
	for (int i = currInd + 1; i < clustersWss.size(); ++i) {

		bool doMerge = checkMergeConditions(clustersWss.at(i).second);
		if (doMerge) {
			float bssClusters = memory->bssBetwenClusters(clustersWss.at(currInd).first, clustersWss.at(i).first);
			if(bssClusters < bssThreshForMerge) {
				
				/*cout << "merging cluster " << clustersWss.at(currInd).first
						<< " with wss: " << clustersWss.at(currInd).second.getNormalizedWss()
						<< " with cluster " << clustersWss.at(i).first
						<< " with wss: " << clustersWss.at(i).second.getNormalizedWss()
						<< endl;
				*/
				//clustersWss.at(i).second.setOperatedOn();
				//return clustersWss.at(i).first;
				// return the cluster index
				return i;
			}
		}
	}
	
	// no suitable cluster was found for merging
	return -1;
}

void KMeans::cleanEmptyClusters(vector<int> &clusters) {
	// delete empty cluters
	for(vector<int>::iterator clts=clusters.begin(); clts != clusters.end(); ++clts) {
		if(memory->getClusterSize(*clts) == 0) {
			memory->deleteCluster(*clts);
		}
	}
}

void KMeans::splitCluster(int &clusterId, vector<int> &touchedClusters) {

	// get the centroid of the cluster to be split
	Histogram<153> &cent = memory->getClusterCentroid(clusterId);
	
	// find the nearest point to the cluster centroid amongst the points assigned to the cluster
	int k = 1;
	vector<int> kIndices(k);
    vector<float> kSqrDistances(k);
	PointCloud<Histogram<153> >::Ptr points (new PointCloud<Histogram<153> >);
	vector<int> mapper;
	memory->getClusterPointsAsCloud(points, mapper, clusterId);
	findNearestPoint(points, cent, kIndices, kSqrDistances, k);
	
	// get the index of the closest point
	int ClosestIdx = mapper.at(kIndices.at(0));
	
	// use point as the centroid of the new cluster and create it
	Histogram<153> &newClusterCent = memory->getFeature(ClosestIdx);
    int nCluster = memory->addCluster(newClusterCent);
	
	vector<int> pointsIdx;
	memory->getClusterPoints(clusterId, pointsIdx);
	
	// unassign the points from the original cluster
    memory->unAssignAllPointsFromCluster(clusterId);
	
	// reassign the points to new clusters
	//assign(pointsIdx);
	// if assgnCluster = -1, then the points may be freely assigned to other clusters
	// other then the original ones. On the other hand if assgnCluster = nCluster, then
	// the points will be assigned to the new cluster only
	int assignCluster = -1;
	assignFeatures(pointsIdx, assignCluster, true, maxIters);
	
	// add modified clusters to the list
	touchedClusters.push_back(clusterId);
	touchedClusters.push_back(nCluster);
}

/*
void KMeans::splitCluster(int &clusterId, vector<int> &touchedClusters) {

	// find a new cluster centroid amongst the features assigned to the cluster to be splitted
	int lFeatIdx = memory->getClusterRandomPoint(clusterId);

	// create a new cluster using the random point as its centroid
	int lCluster = memory->addCluster(memory->getFeature(lFeatIdx));

	// also set the previous cluster centroid to a random point
	// this version of getClusterRandomPoint excludes the value in 'randIdx' from
	// the set of possible returned values (so that the same feature is not used as the centroid for the 2 clusters)
	int rFeatIdx = memory->getClusterRandomPoint(clusterId, lFeatIdx);
	
	// create a new cluster using the random point as its centroid
	int rCluster = memory->addCluster(memory->getFeature(rFeatIdx));

	vector<int> pointsIdx;
	memory->getClusterPoints(clusterId, pointsIdx);
	
	// unassign the points from the original cluster
    memory->unAssignAllPointsFromCluster(clusterId);
	memory->deleteCluster(clusterId);
	
	// reassign the points to new clusters
	//assign(pointsIdx);
	// if assgnCluster = -1, then the points may be freely assigned to other clusters
	// other then the original ones. On the other hand if assgnCluster = nCluster, then
	// the points will be assigned to the new cluster only
	int assignCluster = -1;
	assignFeatures(pointsIdx, assignCluster, true, maxIters);
	
	// add modified clusters to the list
	touchedClusters.push_back(lCluster);
	touchedClusters.push_back(rCluster);
}
*/
void KMeans::mergeClusters(int &lClusterId, int &rClusterId, vector<int> &touchedClusters) {
	
	// get the cenroids of the clusters to be merged together
	Histogram<153> &lCent = memory->getClusterCentroid(lClusterId);
	Histogram<153> &rCent = memory->getClusterCentroid(rClusterId);
	
	// find the center point between the clusters to be merged using a simple average
	// this point will be the centroid of the new cluster
	Histogram<153> newClusterCent = {0};
	for(int i = 0; i < 153; ++i) {
		newClusterCent.histogram[i]  = (lCent.histogram[i] + rCent.histogram[i]) / 2;
	}
	
	// create the new cluster
    int nCluster = memory->addCluster(newClusterCent);
	
	int clusts[] = { lClusterId, rClusterId};
	vector<int> pointsIdx;
	for(int i = 0; i < 2; ++i) {
		vector<int> tmpPointsIdx;
		memory->getClusterPoints(clusts[i], tmpPointsIdx);
		pointsIdx.insert(pointsIdx.end(), tmpPointsIdx.begin(), tmpPointsIdx.end());
	}
	
	// delete the two clusters to be merged and unassign their respective points
    memory->unAssignAllPointsFromCluster(lClusterId);
	memory->unAssignAllPointsFromCluster(rClusterId);
    memory->deleteCluster(lClusterId);
	memory->deleteCluster(rClusterId);
	
	// reassign the points to new clusters
	//assign(pointsIdx);
	// if assgnCluster = -1, then the points may be freely assigned to other clusters
	// other then the original ones. On the other hand if assgnCluster = nCluster, then
	// the points will be assigned to the new cluster only
	int assignCluster = -1;
	assignFeatures(pointsIdx, assignCluster, true, maxIters);
	
	// add modified clusters to the list
	touchedClusters.push_back(nCluster);
}

void KMeans::cleanDegenerateCluster(int &clusterId) {
	
	vector<int> pointsIdx;
	memory->getClusterPoints(clusterId, pointsIdx);
	
	// delete the cluster and unassign its respective points
    memory->unAssignAllPointsFromCluster(clusterId);
	memory->deleteCluster(clusterId);
	
	// reassign the points to new clusters
	//assign(pointsIdx);
	// if assgnCluster = -1, then the points may be freely assigned to other clusters
	// other then the original ones. On the other hand if assgnCluster = nCluster, then
	// the points will be assigned to the new cluster only
	int assignCluster = -1;
	assignFeatures(pointsIdx, assignCluster, true, maxIters);
	
}