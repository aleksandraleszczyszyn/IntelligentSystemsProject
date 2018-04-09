#include <math.h>
#include <algorithm> // sort
#include "Memory.h"
#include "Searcher.h"
#include "kmeans.h"

#include <tr1/unordered_set>

using std::tr1::unordered_set;

using namespace pcl;
using namespace std;

// we need this to compare the spin images with the kd-tree
namespace pcl {
	template <>
	class DefaultPointRepresentation<SpinImage> : public PointRepresentation<SpinImage>
	{
		public:
  			DefaultPointRepresentation ()
  			{
    				nr_dimensions_ = 153;
  			}

  		virtual void copyToFloatArray (const SpinImage &p, float *out) const
  		{
    			for (int i = 0; i < nr_dimensions_; ++i)
      				out[i] = p.histogram[i];
  		}
	};
}
// end

Memory::Memory(){

    fixedSearchRadius = 0.05;

    numFeaturesInMemory = 0;

    numSeenFeatures = 0;

    numSeenObjects = 0;

    numForgottenFeatures = 0;

    numRedundantFeatures = 0;

    // check this value! This is just a test value.
    maxNumFeaturesInMemory = 6000;

    featureRedundancyTreshold = 0.000126;

    smallestDistance = 1000.0;

    forgetFeatures = false;

    DEBUG = false;    

    nForgottenFeaturesByMemoryDecayFactor = 0;

    maxWg = -100000;

    minWg = 100000;

    maxWl = -100000;

    minWl = 100000;

    maxDistance = -100000;

    minDistance = 100000;

    maxSearchRadius = -100000;

    minSearchRadius = 100000;
}

Memory::~Memory()
{}

void Memory::updateCentroids(int pClusterId, int nClusterId, int featIdx) {
    FeatureMetadata &featM = getFeatureMetadata(featIdx);
    //Histogram<153> &feat = getFeature(featIdx);
    // if the previous cluster identifier is -1 that means that
    // the feature has not been assigned to any cluster yet
    // in that case it is unnecessary to updade the centroid
    // of any previous cluster assignment
    if(pClusterId >= 0) {
        // update the previous cluster centroid 'pClusterId' according to this formula
        // update the running totals then calculate (sumFeatures / sumWeight)
        clusters[pClusterId].subtractFromSumWeight(featM.getGlobalWeight());
        clusters[pClusterId].removeFromSumFeatures(featM.getFeature(), featM.getGlobalWeight());
        clusters[pClusterId].recomputeCentroid();
    }

    // update the centroid of the new cluster 'nClusterId' according to this formula
    // update the running totals then calculate (sumFeatures / sumWeight)
    clusters[nClusterId].addToSumWeight(featM.getGlobalWeight());
    clusters[nClusterId].addToSumFeatures(featM.getFeature(), featM.getGlobalWeight());
    clusters[nClusterId].recomputeCentroid();
}

void Memory::syncClusterAssignment(int pClusterId, int nClusterId, int featIdx){

    // remove the feature from the previous assigned cluster
    // if the previous cluster identifier is -1 that means that
    // the feature has not been assigned to any cluster yet
    // in that case it is unnecessary to remove the feature from
    // any previous cluster assignment
    if(pClusterId >= 0) {
        removePointFromCluster(pClusterId, featIdx);
    }

    // assign the feature to the new cluster. Internally the
    // cluster keeps a set of the indexes of the features (points)
    // not the features themselves
    addPointToCluster(nClusterId, featIdx);
}

char Memory::addFeature(pcl::Histogram<153> &feat, int &featureId)
{
    if(!pcl_isfinite(feat.histogram[0])) {
        return 2;
    }

    // add new feature to memory
    featureId = generateFeatureId();
    featuresMetadata.insert(make_pair(featureId, FeatureMetadata(feat)));

    // increment counters
    numSeenFeatures++;
    numFeaturesInMemory++;

    return 1;
}

void Memory::checkRedundancy(vector<int>& featsIdx)
{
	if(!forgetFeatures)
		return;

	//forgetRedundantFeatures(featsIdx);

	// create a snapshot and use that.
    	PointCloud<Histogram<153> >::Ptr memSnapshot(new PointCloud<Histogram<153> >);
	vector<int> mapping;

	for(unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.begin(); 
							  it != featuresMetadata.end(); ++it) {
       		memSnapshot->push_back(it->second.getFeature());
		mapping.push_back(it->first);
	}

	KdTreeFLANN<Histogram<153> > kdtree(false);
	kdtree.setInputCloud(memSnapshot);

	vector<int> knnIds;
	vector<float> knnDistances;
	int foundKnn = 0;
	int categoryId;
	unordered_set<int> toForget;
	unordered_set<float> distances;

	categoryId = getFeatureMetadata(featsIdx.at(0)).getCategory();

	for(size_t i = 0; i < featsIdx.size(); i++){
        	foundKnn = kdtree.nearestKSearch(
					  getFeatureMetadata(featsIdx.at(i)).getFeature()
					, 2
					, knnIds
					, knnDistances);

		for(size_t j = 0; j < knnDistances.size(); j++)
		{
			// prevent redundancy by itself and mutual redundancy
			if(knnDistances.at(j) != 0 && knnDistances.at(j) <= featureRedundancyTreshold
			   &&
		           distances.find(knnDistances.at(j)) == distances.end())
			{
				distances.insert(knnDistances.at(j));
				toForget.insert(featsIdx.at(i));

				// for debug purposes
				if(smallestDistance > knnDistances.at(j))
					smallestDistance = knnDistances.at(j);

				if(maxDistance < knnDistances.at(j))
					maxDistance = knnDistances.at(j);
				//-->

				break;
			}
		}
	}

	if(toForget.size() > 0)
	{

		//cout << fixedSearchRadius << endl;	

		redistributeGlobalWeights(toForget, kdtree, mapping, fixedSearchRadius);

		redistributeLocalWeights(toForget, categoryId);

		// remove features from memory
		for(unordered_set<int>::iterator featId = toForget.begin(); 
						 featId != toForget.end(); ++featId) 
		{
			remFeatureFromMemory(*featId, getFeatureMetadata(*featId));
		}

		numRedundantFeatures += toForget.size();
	}

	/*
	for(size_t i = 0; i < featsIdx.size(); i++){

	    	// GUARD
		unordered_map<int, FeatureMetadata>::iterator iter = featuresMetadata.find(featsIdx.at(i));

		if(iter == featuresMetadata.end()){
		    cout << "FEATURE ID: " << featsIdx.at(i) << endl;
		    throw(std::runtime_error("FEATURE NOT FOUND IN MAP! CALL IN checkRedundancy()."));
		}	
		// --> END GUARD

		isRedundantFeature(iter->second.getFeature(), iter->first);
	}
	*/
}

void Memory::redistributeLocalWeights(unordered_set<int> &toForget, int categoryId)
{
	PointCloud<Histogram<153> >::Ptr memSnapshot(new PointCloud<Histogram<153> >);
	vector<int> mapping;	

	Category &category = categories[categoryId];
	unordered_set<int> &cFeatures = category.getPoints();

	if(toForget.size() > cFeatures.size())
		throw(std::runtime_error("N FEATURES TO FORGET > N FEATURES IN THE CATEGORY!"));

	if(toForget.size() == cFeatures.size()){
		cout << "N FEATURES TO FORGET = N FEATURES IN THE CATEGORY! " 
		     << "CHECK THIS. NO REDISTRIBUTION OF LW WILL OCCUR!" 
		     << endl;
		return;
	}

	for(unordered_set<int>::iterator it = cFeatures.begin(); it != cFeatures.end(); ++it) {
        	memSnapshot->push_back(getFeatureMetadata(*it).getFeature());
	    	mapping.push_back(*it);
	}

	KdTreeFLANN<Histogram<153> > kdtree(false);
	kdtree.setInputCloud(memSnapshot);

	vector<int> knnIds;
	vector<float> knnDistances;
	int foundKnn;
	double sumOfDistances = 0;
	double newLocalWeight = 0;
	FeatureMetadata *aFeatureMetadata;

	for(unordered_set<int>::iterator featId = toForget.begin(); featId != toForget.end(); ++featId) {
		sumOfDistances = 0;

   		foundKnn = kdtree.nearestKSearch(
					  getFeatureMetadata(*featId).getFeature()
					, memSnapshot->size()
					, knnIds
					, knnDistances);

		for(int i = 0; i < knnDistances.size(); i++)
		{
			// filters itself
			if(*featId != mapping.at(knnIds.at(i)))
				sumOfDistances += 1 / sqrt(knnDistances.at(i));
		}
		
		for(int i = 0; i < knnIds.size(); i++)
    		{
			////////////////////////
			// GUARD
			////////////////////////
		    	unordered_map<int, FeatureMetadata>::iterator 
			iter = featuresMetadata.find(mapping.at(knnIds.at(i)));

			if(iter == featuresMetadata.end()){
			    cout << "FEATURE ID: " << mapping.at(knnIds.at(i)) << endl;
			    throw(std::runtime_error(
				"FEATURE NOT FOUND IN MAP! CALL IN redistributeLocalWeights()."));
			}	
			////////////////////////

			if(*featId == mapping.at(knnIds.at(i))){
				continue;
			}

			aFeatureMetadata = &featuresMetadata[mapping.at(knnIds.at(i))];

			newLocalWeight = aFeatureMetadata->getLocalWeight() 
					+ (1 / sqrt(knnDistances.at(i)) / sumOfDistances)
					* getFeatureMetadata(*featId).getLocalWeight();
	
			// for debug
			if(maxWl < newLocalWeight)
			    maxWl = newLocalWeight;

			if(minWl > newLocalWeight)
			    minWl = newLocalWeight;
			//-->

			if(DEBUG)
			    cout << "REDISTRIBUTE LW " << "FEATURE "
					 << mapping.at(knnIds.at(i))
					 << " OLD LW " << aFeatureMetadata->getLocalWeight()
					 << " NEW LW " << newLocalWeight << endl;
		
			aFeatureMetadata->setLocalWeight(newLocalWeight);	
    		}
	}
}

void Memory::redistributeGlobalWeights(	unordered_set<int> &toForget, 
					KdTreeFLANN<Histogram<153> > &kdtree, 
					vector<int> &mapping,
					double searchRadius)
{
	int N = 0;
    	vector<int> pIds;
    	vector<float> pSquaredDistances;

 	double featureGlobalWeight = 0;
	double newFeatureGlobalWeight = 0;
	double sumOfDistances = 0;
	int clusterId;
	Cluster *cluster;	

	FeatureMetadata *aFeatureMetadata;

	for(unordered_set<int>::iterator featId=toForget.begin(); featId != toForget.end(); ++featId) {
		sumOfDistances = 0;	

		N = kdtree.radiusSearch( 
			     getFeatureMetadata(*featId).getFeature()
			   , searchRadius
			   , pIds
			   , pSquaredDistances
			   , 0);

		if(N == 0){
		    cout << "SR: " << searchRadius << " NumFInMEM: " << featuresMetadata.size() << endl;
		    cout << "NO NEIGHBORS FOUND TO REDISTRIBUTE GLOBAL WEIGHTS!" << endl;
		    return;
		}		

		//cout << "N: " << N << endl;	

		if(DEBUG)
	   		cout << "Nº NEIGHBORS TO REDISTRIBUTE WEIGHTS " << N << endl;

		for(int i = 0; i < pSquaredDistances.size(); i++)
	    	{
			if(DEBUG) 
				cout << "DISTANCE " << sqrt(pSquaredDistances.at(i)) << endl;

			// prevent sumimg itself and get: 1/0
			if(*featId != mapping.at(pIds.at(i)))
				sumOfDistances += 1 / sqrt(pSquaredDistances.at(i));
	    	}

		if(DEBUG)
	    		cout << "SUM OF DISTANCES " << sumOfDistances << endl;

		for(int i = 0; i < pIds.size(); i++)
	    	{
			// GUARD
			unordered_map<int, FeatureMetadata>::iterator 
			iter = featuresMetadata.find(mapping.at(pIds.at(i)));

			if(iter == featuresMetadata.end()){
			    cout << "FEATURE ID: " << mapping.at(pIds.at(i)) << endl;
			    throw(std::runtime_error(
				"FEATURE NOT FOUND IN MAP! CALL IN redistributeGlobalWeights()."
			    ));
			}		
			// -->

			if(*featId == mapping.at(pIds.at(i))){
				//cout << "found myself " << pSquaredDistances.at(i) << endl;
				continue;
			}

			aFeatureMetadata = &featuresMetadata[mapping.at(pIds.at(i))];
		
			clusterId = aFeatureMetadata->getCluster();

			featureGlobalWeight = aFeatureMetadata->getGlobalWeight();

			newFeatureGlobalWeight = featureGlobalWeight 
					       + ((1 / sqrt(pSquaredDistances.at(i))) / sumOfDistances)
					       * getFeatureMetadata(*featId).getGlobalWeight();

			// for debug
			if(maxWg < newFeatureGlobalWeight)
			    maxWg = newFeatureGlobalWeight;

			if(minWg > newFeatureGlobalWeight)
			    minWg = newFeatureGlobalWeight;
			//-->

			// GUARD
		       unordered_map<int, Cluster>::iterator it = clusters.find(clusterId);
			if(it == clusters.end()){
			    cout << "CLUSTER ID: " << clusterId << endl;
			    throw(std::runtime_error(
				"CLUSTER NOT FOUND IN MAP! CALL IN redistributeGlobalWeights()."));
			}
			// -->

			cluster = &clusters[clusterId];

			// update cluster centroid
			cluster->removeFromSumFeatures(	aFeatureMetadata->getFeature(),
							featureGlobalWeight);

			cluster->subtractFromSumWeight(featureGlobalWeight);

			cluster->addToSumFeatures(aFeatureMetadata->getFeature(), newFeatureGlobalWeight);
			cluster->addToSumWeight(newFeatureGlobalWeight);

			cluster->recomputeCentroid();

			aFeatureMetadata->setGlobalWeight(newFeatureGlobalWeight);

	    		if(DEBUG)
			    cout << "REDISTRIBUTE GLOBAL WEIGHTS " << "FEATURE "
					 << mapping.at(pIds.at(i)) 
					 << " OLD GW " << featureGlobalWeight 
					 << " NEW GW " << newFeatureGlobalWeight 
					 << endl;
	    	}

	}	
}

void Memory::remFeatureFromMemory(int featureId, FeatureMetadata &featureMetadata)
{
	if(DEBUG)
		cout << "Forgeting point id: " 
			 << featureId << " Cat: " 
			 << featureMetadata.getCategory() << " Cluster: "
			 << featureMetadata.getCluster() << " GW: " 
			 << featureMetadata.getGlobalWeight() << " LW: "
			 << featureMetadata.getLocalWeight()
			 << endl;


	unordered_map<int, Cluster>::iterator it = clusters.find(featureMetadata.getCluster());
	if(it == clusters.end()){
		cout << "CLUSTER ID: " << featureMetadata.getCluster() << endl;
		throw(std::runtime_error("CLUSTER NOT FOUND IN MAP! CALL IN forgetFeature()."));
	}

	// update cluster centroid
	Cluster &cluster = clusters[featureMetadata.getCluster()];

	cluster.removeFromSumFeatures(featureMetadata.getFeature(), featureMetadata.getGlobalWeight());
	cluster.subtractFromSumWeight(featureMetadata.getGlobalWeight());

	removePointFromCluster2(featureMetadata.getCluster(), featureId);
	removePointFromCategory2(featureMetadata.getCategory(), featureId);

	// remove feature from memory
	removeFeatureFromMemory(featureId);

	cluster.recomputeCentroid();

	numForgottenFeatures++;
	numFeaturesInMemory--;
}

Histogram<153>& Memory::getFeature(const int &key) {

    unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.find(key);
    if(it == featuresMetadata.end()){
	cout << "FEATURE ID " << key << endl;
	throw(std::runtime_error("FEATURE NOT FOUND IN MAP! IN CALL getFeature()."));
    }

    // return the feature at the specified key
    return featuresMetadata[key].getFeature();
}

const FeatureMetadata* Memory::getFeatureMetadata2(const int &key)
{
    unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.find(key);
    if(it == featuresMetadata.end()){
	return NULL;
    }

    return &featuresMetadata[key];
}

FeatureMetadata &Memory::getFeatureMetadata(const int &key) {

    unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.find(key);
    if(it == featuresMetadata.end()){
		cout << "FEATURE ID " << key << endl;
		throw(std::runtime_error("FEATURE NOT FOUND IN MAP!"));
    }

    // return the metadata of the feature at the specified key
    return featuresMetadata[key];
}

void Memory::getCentroids(
	PointCloud<Histogram<153> >::Ptr &centroids, vector<int> &mapper, unordered_map<int, int> &revMapper) {

    // cluster information is held in a map structure (a dictionary)
    // the map is iterated in order to build a vector holding all the
    // cluster centroids
    if(clusters.empty()) {
        return;
    }

    for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
        centroids->push_back(clt->second.getCentroid());
        // keep a structure to map from the centroids vector position to the
        // respective cluster identifiers
        mapper.push_back(clt->first);
		revMapper[clt->first] = mapper.size() - 1;
    }
}

void Memory::getClusterPointsAsCloud(PointCloud<Histogram<153> >::Ptr &points, vector<int> &pointsMapper, int &clusterId) {

	if(clusters.empty()) {
		return;
	}
	
	Cluster clt = clusters[clusterId];
	for(unordered_set<int>::iterator featIdx=clt.getPoints().begin(); featIdx != clt.getPoints().end(); ++featIdx) {
		FeatureMetadata &featM = getFeatureMetadata(*featIdx);
		Histogram<153> &feat = featM.getFeature();
		Histogram<153> nFeat = {0};
		for(int i = 0; i < 153; ++i) {
			nFeat.histogram[i] = feat.histogram[i] * featM.getGlobalWeight();
		}
		points->push_back(nFeat);
		pointsMapper.push_back(*featIdx);
	}
}

Histogram<153> &Memory::getClusterCentroid(int &clusterId) {
    return clusters[clusterId].getCentroid();
}

int Memory::addCluster(Histogram<153> &centroid) {
    int nCluster = generateClusterId();
    Cluster cluster(nCluster, centroid);
    clusters.insert(make_pair(nCluster, cluster));
    
    return nCluster;
}

void Memory::deleteCluster(int &clusterId) {
    clusters.erase(clusterId);
}

void Memory::addPointToCluster(int &clusterId, int &featIdx) {
    clusters[clusterId].addPoint(featIdx);
}

void Memory::removePointFromCluster(int &clusterId, int &featIdx) {
    clusters[clusterId].removePoint(featIdx);
}

void Memory::removeFeatureFromMemory(int featureId)
{
    if(DEBUG)
        cout << "REMOVE FEATURE FROM MEMORY: " << featureId << endl;

    unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.find(featureId);
    if(it == featuresMetadata.end()){
	cout << "FEATURE ID " << featureId << endl;
	throw(std::runtime_error("REMOVING FEATURE NOT FOUND IN MAP!"));
    }

    featuresMetadata.erase(featureId);
}

void Memory::removePointFromCluster2(int clusterId, int featIdx) {
    clusters[clusterId].removePoint(featIdx);
}

void Memory::unAssignAllPointsFromCluster(int &clusterId) {

    // set all features previously assigned to this cluster to cluster -1
    // that is, set the features as unassigned to any cluster
    unordered_set<int> &pointsIdx = clusters[clusterId].getPoints();
    for(unordered_set<int>::iterator featIdx=pointsIdx.begin(); featIdx != pointsIdx.end(); ++featIdx) {
        getFeatureMetadata(*featIdx).unAssignFromCluster();
    }
    
    // clear points from cluster
    clusters[clusterId].clearPoints();
}

void Memory::setClusterCentroid(int &clusterId, int &featIdx) {
    clusters[clusterId].setCentroid(getFeature(featIdx));
}

void Memory::resetClusterTotals(int &clusterId) {
    clusters[clusterId].resetSumWeight();
    clusters[clusterId].resetSumFeatures();
}

int &Memory::getBiggerCluster(int &lClusterId, int &rClusterId) {

    return ((clusters[lClusterId].size() >  clusters[rClusterId].size()) ? lClusterId : rClusterId);
}

void Memory::getClusterPoints(int &clusterId, vector<int> &pointsIdx) {
    return clusters[clusterId].getPoints(pointsIdx);
}

int Memory::getClusterSize(int &clusterId) {
	return clusters[clusterId].size();
}

float Memory::bssBetwenClusters(int& lCluster, int& rCluster) {
    float bss = 0.0f;
	float eucl = 0.0f;
	
    Histogram<153> lCentroid = clusters[lCluster].getCentroid();
	Histogram<153> rCentroid = clusters[rCluster].getCentroid();
    
	for(int i = 0; i < 153; ++i) {
		float dist = lCentroid.histogram[i] - rCentroid.histogram[i];
		eucl += dist * dist;
	}
	
	bss = sqrt(eucl);

    return bss;
}

float Memory::bss(unordered_map<int, BssBag> &clustersBss) {
    float bss = 0.0f;
    int nClusters = clusters.size();
    // compute the 'between sum of squares' for the clusters (using euclidean distance)
    // compute feature dataset global mean
    Histogram<153> gCentroid = {0};

	float sumWeight = 0;
    
    for(unordered_map<int, FeatureMetadata>::iterator featM=featuresMetadata.begin(); featM != featuresMetadata.end(); ++featM) {
        Histogram<153> &feat = featM->second.getFeature();
        sumWeight += featM->second.getGlobalWeight();
        for(int i = 0; i < 153; ++i) {
            gCentroid.histogram[i] += feat.histogram[i] * featM->second.getGlobalWeight();
        }
    }
    
    for(int i = 0; i < 153; ++i) {
        gCentroid.histogram[i] /= sumWeight;
    }

    //cout << "g cent " << gCentroid << endl;
    
    // for each cluster
    for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
        float clusterBss = 0.0f;
		//nClusters += 1;
        Histogram<153> &centroid = clt->second.getCentroid();
        float eucl = 0.0f;
		BssBag measureValues;
        // compute the euclidean distance between the cluster centroid and the global centroid
        for(int i = 0; i < 153; ++i) {
            float dist = centroid.histogram[i] - gCentroid.histogram[i];
            eucl += dist * dist;
        }
        clusterBss = clt->second.size() * sqrt(eucl);
		measureValues.setId(clt->second.getIdentifier());
		measureValues.setBss(clusterBss);
		measureValues.setCount(clt->second.size());
		clustersBss.insert(make_pair(clt->second.getIdentifier(), measureValues));
        //clustersBss.insert(make_pair(clt->second.getIdentifier(), clusterBss));
        bss += clusterBss;
    }

    /*
    // store pairwise bss distances between clusters
    for(int i = 0; i < clustersBss.size() - 1; ++i) {
        for(int j = i + 1; j < clustersBss.size(); ++j) {
    }
    */
	//cout << "normalized BSS " << bss / nClusters<< endl;
    return bss;
}

float Memory::wss(vector<pair<int, WssBag> > &clustersWss) {

    float dataSetWss = 0.0f;
	int datasetNSamples = 0;
	
    // compute the 'whithin sum of squares' for the clusters (using euclidean distance)
    for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
        float clusterWss = 0.0f;
		float minDistance = 0.0f;
		float maxDistance = 0.0f;
		//float radius = 0.0f;
		//float clusterRadius = 0.0f;
		datasetNSamples += clt->second.size();
        Histogram<153> &centroid = clt->second.getCentroid();
		WssBag wssMeasureValues;
        // for each feature assigned to the cluster
        for(unordered_set<int>::iterator featIdx=clt->second.getPoints().begin(); featIdx != clt->second.getPoints().end(); ++featIdx) {
            FeatureMetadata &featM = getFeatureMetadata(*featIdx);
            Histogram<153> &feat = featM.getFeature();
            float distSum = 0.0f;
            // compute the euclidean distance between the feature and the centroid of the cluster
            for(int i = 0; i < 153; ++i) {
                float dist = (feat.histogram[i] * featM.getGlobalWeight()) - centroid.histogram[i];
                distSum += dist * dist;
            }
			//radius += eucl;
			distSum = sqrt(distSum);
			maxDistance = distSum > maxDistance ? distSum : maxDistance;
			minDistance = minDistance == 0 ? distSum : ( distSum < minDistance ? distSum : minDistance);
            clusterWss += distSum;
			//clusterRadius += eucl;
        }
		wssMeasureValues.setId(clt->second.getIdentifier());
		wssMeasureValues.setWss(clusterWss);
		wssMeasureValues.setCount(clt->second.size());
		wssMeasureValues.setMinDistance(minDistance);
		wssMeasureValues.setMaxDistance(maxDistance);
		//wssMeasureValues.setAsBssCandidate(clusterWss < wssThreshForMerge);
		//measureValues.setClusterRadius(sqrt(radius / clt->second.size()));
        // for each cluster keeps the cluster identifier as well as the clusters wss
		clustersWss.push_back(make_pair(clt->second.getIdentifier(), wssMeasureValues));
        //clustersWss.push_back(make_pair(clt->second.getIdentifier(), clusterWss));
        dataSetWss += clusterWss;
    }
	
	/*for(vector<pair<int, WssBag> >::iterator clt=clustersWss.begin(); clt != clustersWss.end(); ++clt) {
		cout << "clustersWss before sort: cluster id " << clt->first
				<< " with aize " << clt->second.getCount()
				<< " cluster wss: " << clt->second.getWss()
				<<  " normalized " << clt->second.getNormalizedWss() << endl;
	}*/
    // sort the wss vector in descending order (bigger wss values first)
    //sort(clustersWss.begin(), clustersWss.end(), descending());
	/*
	for(vector<pair<int, WssBag> >::iterator clt=clustersWss.begin(); clt != clustersWss.end(); ++clt) {
		cout << "clustersWss : cluster id " << clt->first 
				<< " with size " << clt->second.getCount()
				//<< " cluster wss: " << clt->second.getWss()
				<< " min distance: " << clt->second.getMinDistance()
				<< " max distance: " << clt->second.getMaxDistance()
				<< " normalized " << clt->second.getNormalizedWss()
				//<< " candidate o merging " << clt->second.isBssCandidate()
				//<< " radius " << clt->second.getClusterRadius()
				//<< " density " << clt->second.getClusterDensity() 
				<< endl;
	}
	*/
	
	//cout << "normalized WSS " << wss / nSamples << endl;
    return (dataSetWss / datasetNSamples); // / clusters.size();
	//return wss;
}

const std::string& Memory::getCategoryName(int categoryId)
{
    return categories[categoryId].getName();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
//View
////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<double>* Memory::getViewHistogram(std::vector<int>& featsIdx, vector<int> &redundancyFeatsIdx)
{
    throw(std::runtime_error("USE CategoryRecognizer::getViewHistogram() INSTEAD!"));
}

int Memory::getViewCategory(std::vector<double>& viewHistogram)
{
    throw(std::runtime_error("USE CategoryRecognizer::getViewCategory() INSTEAD!"));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Category
////////////////////////////////////////////////////////////////////////////////////////////////////////

int Memory::getCategoryId(const string &categoryName)
{
    unordered_map<string, int>::iterator it = categoriesIdMap.find(categoryName);
    if(it == categoriesIdMap.end()){
	cout << "CATEGORY " << categoryName << endl;
	throw(std::runtime_error("CATEGORY NOT FOUND IN MAP!"));
    }
    else{
	return it->second;
    }    
}

int Memory::addCategory(string categoryName){

    int categoryId;

    unordered_map<string, int>::iterator it = categoriesIdMap.find(categoryName);

    if(it == categoriesIdMap.end()){
	categoryId = generateCategoryId();

	categories.insert(make_pair(categoryId, Category(categoryId, categoryName)));

	categoriesIdMap[categoryName] = categoryId;
    }
    else{
	categoryId = it->second;
    }

    return categoryId;
}

void Memory::addPointToCategory(int categoryId, int featIdx) {
    
    unordered_map<int, Category>::iterator it = categories.find(categoryId);
    if(it == categories.end()){
	cout << "CATEGORY ID: " << categoryId << endl;
	throw(std::runtime_error("CATEGORY NOT FOUND IN MAP! CALL IN addPointToCategory()."));
    }

    unordered_map<int, FeatureMetadata>::iterator it2 = featuresMetadata.find(featIdx);
    if(it2 == featuresMetadata.end()){
	cout << "FEATURE ID: " << featIdx << endl;
	throw(std::runtime_error("FEATURE NOT FOUND IN MAP! CALL IN addPointToCategory()."));
    }
    
    categories[categoryId].addPoint(featIdx);
    featuresMetadata[featIdx].assignToCategory(categoryId);
}

void Memory::assignViewToCategory(std::vector<int>& featsIdx, int categoryId)
{
    for(size_t i = 0; i < featsIdx.size(); i++){
	addPointToCategory(categoryId, featsIdx.at(i));
    }
}

void Memory::removePointFromCategory(int &categoryId, int &featIdx) {

    unordered_map<int, Category>::iterator it = categories.find(categoryId);
    if(it == categories.end()){
	cout << "CATEGORY ID: " << categoryId << endl;
	throw(std::runtime_error("CATEGORY NOT FOUND IN MAP! CALL IN removePointFromCategory()."));
    }

    unordered_map<int, FeatureMetadata>::iterator it2 = featuresMetadata.find(featIdx);
    if(it2 == featuresMetadata.end()){
	cout << "FEATURE ID: " << featIdx << endl;
	throw(std::runtime_error("FEATURE NOT FOUND IN MAP! CALL IN removePointFromCategory()."));
    }

    categories[categoryId].removePoint(featIdx);

    // unassign feature from category in metadata main memory structure
    featuresMetadata[featIdx].unAssignFromCategory();
}

void Memory::removePointFromCategory2(int categoryId, int featIdx) {

    unordered_map<int, Category>::iterator it = categories.find(categoryId);
    if(it == categories.end()){
	cout << "CATEGORY ID: " << categoryId << endl;
	throw(std::runtime_error("CATEGORY NOT FOUND IN MAP! CALL IN removePointFromCategory2()."));
    }

    unordered_map<int, FeatureMetadata>::iterator it2 = featuresMetadata.find(featIdx);
    if(it2 == featuresMetadata.end()){
	cout << "FEATURE ID: " << featIdx << endl;
	throw(std::runtime_error("FEATURE NOT FOUND IN MAP! CALL IN removePointFromCategory2()."));
    }

    categories[categoryId].removePoint(featIdx);

    // unassign feature from category in metadata main memory structure
    featuresMetadata[featIdx].unAssignFromCategory();
}

void Memory::printCategoriesInfo()
{
    for(unordered_map<int, Category>::iterator it = categories.begin(); it != categories.end(); ++it) {
        cout << it->first << ": " << it->second.toString() << endl;
    }
}

void Memory::addTPToCategory(int &categoryId) {
    categories[categoryId].addTP();
}

void Memory::addFPToCategory(int &categoryId) {
    categories[categoryId].addFP();
}

void Memory::addFNToCategory(int &categoryId) {
    categories[categoryId].addFN();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
//Memory Management
/*
int Memory::getFeatureToForgetByDecayFactor()
{
    double featureGlobalWeight;
    double smallestFeatureGlobalWeight = 1000000;
    int featureId;
    
    for(unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.begin(); it != featuresMetadata.end(); ++it) {
		featureGlobalWeight = it->second.getGlobalWeight();

		// cluster and category check is a temporary work around.
        if(it->second.getCluster() != -1 && it->second.getCategory() != -1 && smallestFeatureGlobalWeight > featureGlobalWeight){
	    	smallestFeatureGlobalWeight = featureGlobalWeight;
	    	featureId = it->first;
		}
    }

    if(DEBUG)
        cout << "FORGET FEATURE: " << featureId 
	     	 << " GLOBAL WEIGHT: " << smallestFeatureGlobalWeight << endl;

    return featureId;
}
*/
void Memory::applyMemoryDecay()
{
	numSeenObjects++;

	/*
	cout 	<< "M: " << numFeaturesInMemory 
		<< " R: " << numRedundantFeatures 
		<< " C: " << clusters.size()
		<< " FF: " << numForgottenFeatures << endl;
	*/

	// check if the forgetting mechanism is being used.
	if(!forgetFeatures || featuresMetadata.size() == 0)
		return;
 
	Cluster *cluster;
    	int clusterId;
    	double featureGlobalWeight;
    	double newFeatureGlobalWeight;
    	double featureLocalWeight;
    	double newFeatureLocalWeight;

	unordered_set<int> featuresToRemove;

    	for(unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.begin(); 
					    	it != featuresMetadata.end(); ++it) {
	
		clusterId = it->second.getCluster();

		featureGlobalWeight = it->second.getGlobalWeight();
		featureLocalWeight = it->second.getLocalWeight();

		newFeatureGlobalWeight = applyMemoryDecayFactor(featureGlobalWeight);
		newFeatureLocalWeight = applyMemoryDecayFactor(featureLocalWeight);

		// for debug
		if(maxWg < newFeatureGlobalWeight)
			maxWg = newFeatureGlobalWeight;

		if(minWg > newFeatureGlobalWeight)
			minWg = newFeatureGlobalWeight;

		if(maxWl < newFeatureLocalWeight)
			maxWl = newFeatureLocalWeight;

		if(minWl > newFeatureLocalWeight)
			minWl = newFeatureLocalWeight;
		//-->

		if(DEBUG)
			cout << "MEMORY DECAY: " << "FEATURE: " << it->first 
				 << " OLD GLOBAL WEIGHT: " << featureGlobalWeight 
				 << " NEW GLOBAL WEIGHT WITH DECAY: " << newFeatureGlobalWeight 
				 << " OLD LOCAL WEIGHT: " << featureLocalWeight 
				 << " NEW LOCAL WEIGHT WITH DECAY: " << newFeatureLocalWeight 
				 << endl;
		
		// forget this feature
		if(forgetFeatures && newFeatureGlobalWeight <= memoryDecayTreshold){
			//featuresToRemove.push_back(it->first);
			featuresToRemove.insert(it->first);
			continue;
		}

		// apply memory decay factor
		it->second.setGlobalWeight(newFeatureGlobalWeight);
		it->second.setLocalWeight(newFeatureLocalWeight);

		unordered_map<int, Cluster>::iterator it2 = clusters.find(clusterId);
		if(it2 == clusters.end()){
			cout << "CLUSTER ID: " << clusterId << endl;
			throw(std::runtime_error(
				"CLUSTER NOT FOUND IN MAP! CALL IN applyMemoryDecay()."));
		}

		// update the cluster centroid
		cluster = &clusters[clusterId];

		cluster->removeFromSumFeatures(it->second.getFeature(), featureGlobalWeight);
		cluster->subtractFromSumWeight(featureGlobalWeight);

		cluster->addToSumFeatures(it->second.getFeature(), newFeatureGlobalWeight);
		cluster->addToSumWeight(newFeatureGlobalWeight);

		cluster->recomputeCentroid();
    	}	

    	// forget features with a global weight bellow a given threshold
    	//for(int i = 0; i < featuresToRemove.size(); i++){
	if(featuresToRemove.size() > 0){
		
		// create a snapshot and use that.
	    	PointCloud<Histogram<153> >::Ptr memSnapshot(new PointCloud<Histogram<153> >);
		vector<int> mapping;

		for(unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.begin(); 
								  it != featuresMetadata.end(); ++it) {
	       		memSnapshot->push_back(it->second.getFeature());
			mapping.push_back(it->first);
		}

		KdTreeFLANN<Histogram<153> > kdtree(false);
		kdtree.setInputCloud(memSnapshot);

		redistributeGlobalWeights(featuresToRemove, kdtree, mapping, fixedSearchRadius);

		//redistributeLocalWeights(featuresToRemove, categoryId);

		// remove features from memory
		for(unordered_set<int>::iterator featId = featuresToRemove.begin(); 
						 featId != featuresToRemove.end(); ++featId) 
		{
			redistributeLocalWeights(*featId, getFeatureMetadata(*featId));
			remFeatureFromMemory(*featId, getFeatureMetadata(*featId));
		}

		//numRedundantFeatures += toForget.size();		

    		//forgetFeature(featuresToRemove.at(i));
		nForgottenFeaturesByMemoryDecayFactor += featuresToRemove.size();
    	}
}

void Memory::redistributeGlobalWeights(	int featureId, 
					FeatureMetadata &featureMetadata, 
					double searchRadius)
{
	int returnedMatches = 0;
	PointCloud<Histogram<153> >::Ptr points(new PointCloud<Histogram<153> >);
	vector<int> mapping;
	
	// Establish the search domain
    	// if there are some clusters try to reduce the search domain
    	if(clusters.size() > 0){
    		PointCloud<Histogram<153> >::Ptr centroids(new PointCloud<Histogram<153> >);
    		unordered_map<int, int> clustersIdxCorrespondence;

   	 	for(unordered_map<int, Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it) {
        		centroids->push_back(it->second.getCentroid());
			clustersIdxCorrespondence[centroids->size() - 1] = it->first;
    		}    

    		vector<int> idxNKNSearch;
    		vector<float> NKNSquaredDistance;

    		// find the k closests clusters centroids, k = 3
    		int nMatches = searcher.KDtreeSPMatch(
			      featureMetadata.getFeature()
			    , centroids, 3
			    , idxNKNSearch
			    , NKNSquaredDistance, false);    

    		if(nMatches == 0){
	    		throw(std::runtime_error(
				"UNEXPECTED: redistributeGlobalWeights(), No closest cluster found!"));
    		}

    		int clusterId;

    		Cluster *cluster;
    		unordered_set<int> *clusterPointIdxs;
    	
    		for(int i = 0; i < idxNKNSearch.size(); i++){
			clusterId = clustersIdxCorrespondence[idxNKNSearch.at(i)];

        		unordered_map<int, Cluster>::iterator it = clusters.find(clusterId);
        		if(it == clusters.end()){
	    	    		cout << "CLUSTER ID: " << clusterId << endl;
	    	    		throw(std::runtime_error(
				"CLUSTER NOT FOUND IN MAP! CALL IN redistributeGlobalWeights()."));
        		}

	        	cluster = &clusters[clusterId];

			clusterPointIdxs = &cluster->getPoints();

			// get cluster points/features
			for(unordered_set<int>::iterator it = clusterPointIdxs->begin(); 
						it != clusterPointIdxs->end(); ++it) {
				if(*it != featureId){
            				points->push_back(getFeatureMetadata(*it).getFeature());
	    	    			mapping.push_back(*it);
				}
			}	
		}
    	// if no clusters are available
    	}else{
		throw(std::runtime_error(
			"CLUSTERS NOT FOUND IN MAP! CALL IN redistributeGlobalWeights()."));
    	}	
	//-->
	
    	vector<int> pointsIdxs;
    	vector<float> pointsSquaredDistance;

	returnedMatches = searcher.KDtreeRadiusSearch(featureMetadata.getFeature()
			, points
			, searchRadius
			, 0
			, pointsIdxs
			, pointsSquaredDistance);

	if(returnedMatches == 0){
	    cout << "SR " << searchRadius << " " << featuresMetadata.size() << endl;
	    cout << "NO NEIGHBORS FOUND TO REDISTRIBUTE GLOBAL WEIGHTS!" << endl;
	    return;
	}

    	if(DEBUG)
	    cout << "Nº NEIGHBORS TO REDISTRIBUTE WEIGHTS " << returnedMatches << endl;

	double newFeatureGlobalWeight = 0;
 	double featureGlobalWeight = 0;
	FeatureMetadata *aFeatureMetadata;
	double sumOfDistances = 0;
	int clusterId;
	Cluster *cluster;

	for(int i = 0; i < pointsSquaredDistance.size(); i++)
    	{
		if(DEBUG) cout << "DISTANCE " << sqrt(pointsSquaredDistance.at(i)) << endl;

		sumOfDistances += 1 / sqrt(pointsSquaredDistance.at(i));
    	}

    	if(DEBUG)
	    cout << "SUM OF DISTANCES " << sumOfDistances << endl;

	for(int i = 0; i < pointsIdxs.size(); i++)
    	{
		// GUARD
		unordered_map<int, FeatureMetadata>::iterator 
		iter = featuresMetadata.find(mapping.at(pointsIdxs.at(i)));

		if(iter == featuresMetadata.end()){
		    cout << "FEATURE ID: " << mapping.at(pointsIdxs.at(i)) << endl;
		    throw(std::runtime_error(
			"FEATURE NOT FOUND IN MAP! CALL IN redistributeGlobalWeights()."
		    ));
		}		
		// -->

		aFeatureMetadata = &featuresMetadata[mapping.at(pointsIdxs.at(i))];
		
		clusterId = aFeatureMetadata->getCluster();

		featureGlobalWeight = aFeatureMetadata->getGlobalWeight();

		newFeatureGlobalWeight = featureGlobalWeight 
				       + ((1 / sqrt(pointsSquaredDistance.at(i))) / sumOfDistances)
				       * featureMetadata.getGlobalWeight();

		// for debug
		if(maxWg < newFeatureGlobalWeight)
		    maxWg = newFeatureGlobalWeight;

		if(minWg > newFeatureGlobalWeight)
		    minWg = newFeatureGlobalWeight;
		//-->

		if(clusterId == -1)
			throw(std::runtime_error(
				"CLUSTER = -1! CALL IN redistributeGlobalWeights()."));

		// GUARD
	       unordered_map<int, Cluster>::iterator it = clusters.find(clusterId);
	        if(it == clusters.end()){
		    cout << "CLUSTER ID: " << clusterId << endl;
		    throw(std::runtime_error(
			"CLUSTER NOT FOUND IN MAP! CALL IN redistributeGlobalWeights()."));
	        }
		// -->

		cluster = &clusters[clusterId];

		// update cluster centroid
		cluster->removeFromSumFeatures(aFeatureMetadata->getFeature(),
						       featureGlobalWeight);

		cluster->subtractFromSumWeight(featureGlobalWeight);

		cluster->addToSumFeatures(aFeatureMetadata->getFeature(), newFeatureGlobalWeight);
		cluster->addToSumWeight(newFeatureGlobalWeight);

		cluster->recomputeCentroid();

		aFeatureMetadata->setGlobalWeight(newFeatureGlobalWeight);

    		if(DEBUG)
		    cout << "REDISTRIBUTE GLOBAL WEIGHTS " << "FEATURE "
				 << mapping.at(pointsIdxs.at(i)) 
				 << " OLD GW " << featureGlobalWeight 
				 << " NEW GW " << newFeatureGlobalWeight 
				 << endl;
    	}
}

void Memory::redistributeLocalWeights(int featureId, FeatureMetadata &featureMetadata)
{
	PointCloud<Histogram<153> >::Ptr points(new PointCloud<Histogram<153> >);
	vector<int> correspondencies;
	int returnedMatches = 0;
	int categoryID = featureMetadata.getCategory();
	int nPoints;

	if(DEBUG)
	    cout << "REDISTRIBUTING LOCAL WEIGHTS: CATEGORY " << categoryID << endl;

	/*
	if(categoryID == -1){
	     throw(std::runtime_error("redistributeLocalWeights() category -1!"));
	}
	*/
		
	//if(categoryID != -1){
	Category &category = categories[categoryID];
	
	unordered_set<int> &categoryFeatures = category.getPoints();

	nPoints = categoryFeatures.size() - 1;

	// no points to redistribute weights to.
	if(nPoints < 1)
	    return;

	for(unordered_set<int>::iterator it = categoryFeatures.begin(); 
					 it != categoryFeatures.end(); ++it) {
		if(*it != featureId){
	        	points->push_back(getFeatureMetadata(*it).getFeature());
	    		correspondencies.push_back(*it);
		}
	}

    	vector<int> pointsIdxs;
    	vector<float> pointsSquaredDistance;

    	returnedMatches = searcher.KDtreeSPMatch(	
						  featureMetadata.getFeature()
					    	, points
						, nPoints
				    		, pointsIdxs
				    		, pointsSquaredDistance, false);

	if(returnedMatches != nPoints){
	     throw(std::runtime_error("redistributeLocalWeights() Wrong number of points returned!"));
	}

	double newLocalWeight = 0;
	FeatureMetadata *aFeatureMetadata;

	double sumOfDistances = 0;

	for(int i = 0; i < pointsSquaredDistance.size(); i++)
    	{
		sumOfDistances += 1 / sqrt(pointsSquaredDistance.at(i));
    	}

	for(int i = 0; i < pointsIdxs.size(); i++)
    	{
		////////////////////////
		// GUARD
		////////////////////////
	    	unordered_map<int, FeatureMetadata>::iterator 
		iter = featuresMetadata.find(correspondencies.at(pointsIdxs.at(i)));

		if(iter == featuresMetadata.end()){
		    cout << "FEATURE ID: " << correspondencies.at(pointsIdxs.at(i)) << endl;
		    throw(std::runtime_error(
			"FEATURE NOT FOUND IN MAP! CALL IN redistributeLocalWeights()."));
		}	
		////////////////////////

		aFeatureMetadata = &featuresMetadata[correspondencies.at(pointsIdxs.at(i))];

		newLocalWeight = aFeatureMetadata->getLocalWeight() 
				+ (1 / sqrt(pointsSquaredDistance.at(i)) / sumOfDistances)
				* featureMetadata.getLocalWeight();
	
		// for debug
		if(maxWl < newLocalWeight)
		    maxWl = newLocalWeight;

		if(minWl > newLocalWeight)
		    minWl = newLocalWeight;
		//-->

		if(DEBUG)
		    cout << "REDISTRIBUTE LW " << "FEATURE "
				 << correspondencies.at(pointsIdxs.at(i))
				 << " OLD LW " << aFeatureMetadata->getLocalWeight()
				 << " NEW LW " << newLocalWeight << endl;
		
		aFeatureMetadata->setLocalWeight(newLocalWeight);	
    	}
}

/*
void Memory::redistributeWeights(int featureId)
{
	////////////////
    // GUARD
    ////////////////
	unordered_map<int, FeatureMetadata>::iterator iter = featuresMetadata.find(featureId);
	if(iter == featuresMetadata.end()){
	    cout << "FEATURE ID: " << featureId << endl;
	    throw(std::runtime_error("FEATURE NOT FOUND IN MAP! CALL IN redistributeWeights()."));
	}	
	////////////////

	FeatureMetadata &featureMetadata = featuresMetadata[featureId];

	double searchRadius = 0;
	
	// K closest clusters mean distance to the feature, k = 2
	searchRadius = getSearchRadius(featureId, featureMetadata, 2);

	if(DEBUG)	
	    cout << "SEARCH RADIUS " << searchRadius << endl;

	// for debug purposes
    if(maxSearchRadius < searchRadius)
	    maxSearchRadius = searchRadius;

    if(minSearchRadius > searchRadius)
	    minSearchRadius = searchRadius;
	//-->

 	redistributeGlobalWeights(featureId, featureMetadata, searchRadius);	
	
	// Waiting for architecture changes.
	redistributeLocalWeights(featureId, featureMetadata);
}
*/

void Memory::forgetFeature(int featureId/*, FeatureMetadata &featureMetadata*/)
{
	////////////////
    	// GUARD
    	////////////////
	unordered_map<int, FeatureMetadata>::iterator iter = featuresMetadata.find(featureId);
	if(iter == featuresMetadata.end()){
	    cout << "FEATURE ID: " << featureId << endl;
	    throw(std::runtime_error("FEATURE NOT FOUND IN MAP! CALL IN forgetFeature()."));
	}	
	////////////////

	FeatureMetadata &featureMetadata = featuresMetadata[featureId];

	if(DEBUG)
		cout << "Forgeting point id: " 
			 << featureId << " Cat: " 
			 << featureMetadata.getCategory() << " Cluster: "
			 << featureMetadata.getCluster() << " GW: " 
			 << featureMetadata.getGlobalWeight() << " LW: "
			 << featureMetadata.getLocalWeight()
			 << endl;

	double searchRadius = 0;
	
	// K closest clusters mean distance to the feature, k = 2
	searchRadius = getSearchRadius(featureId, featureMetadata, 2);

	if(DEBUG)	
	    cout << "SEARCH RADIUS " << searchRadius << endl;

	/*
	if(searchRadius == -1)
	    return;
	*/

 	redistributeGlobalWeights(featureId, featureMetadata, searchRadius);	
	
	redistributeLocalWeights(featureId, featureMetadata);

	unordered_map<int, Cluster>::iterator it = clusters.find(featureMetadata.getCluster());
	if(it == clusters.end()){
		cout << "CLUSTER ID: " << featureMetadata.getCluster() << endl;
		throw(std::runtime_error("CLUSTER NOT FOUND IN MAP! CALL IN forgetFeature()."));
	}

	// update cluster centroid
	Cluster &cluster = clusters[featureMetadata.getCluster()];

	cluster.removeFromSumFeatures(featureMetadata.getFeature(), featureMetadata.getGlobalWeight());
	cluster.subtractFromSumWeight(featureMetadata.getGlobalWeight());

	removePointFromCluster2(featureMetadata.getCluster(), featureId);
	removePointFromCategory2(featureMetadata.getCategory(), featureId);

	// remove feature from memory
	removeFeatureFromMemory(featureId);

	cluster.recomputeCentroid();

	numForgottenFeatures++;
	numFeaturesInMemory--;
}

double Memory::getSearchRadius(int featureId, FeatureMetadata &featureMetadata, int k)
{
	int returnedMatches = 0;
	PointCloud<Histogram<153> >::Ptr centroids(new PointCloud<Histogram<153> >);

	// if no cluster exist what should be done?
	if(clusters.size() == 0){
	    //return 0.1;
	    throw(std::runtime_error(
		"NO CLUSTERS FOUND! CALL IN getSearchRadius()."
	    ));
	}

	for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
		centroids->push_back(clt->second.getCentroid());
	}

	vector<int> pointsIdxs;
	vector<float> pointsSquaredDistance;

	returnedMatches = searcher.KDtreeSPMatch(featureMetadata.getFeature()
				    , centroids, k
				    , pointsIdxs
				    , pointsSquaredDistance, false);

	if(returnedMatches == 0){
	    throw(std::runtime_error(
		"NO CLUSTERS FOUND! CALL IN getSearchRadius()."
	    ));
	}
	
	// guard.
	if(returnedMatches != pointsSquaredDistance.size()){
	    throw(std::runtime_error(
		"UNEXPECTED NUMBER OF RETURNED MATCES getSearchRadius()."
	    ));
	}

	double searchRadius = 0;

	for(int i = 0; i < returnedMatches; i++)
   	{
		searchRadius += sqrt(pointsSquaredDistance.at(i));
		//searchRadius += pointsSquaredDistance.at(i);
   	}

	//cout << returnedMatches << endl;

	// get a mean
	searchRadius /= returnedMatches;	
	
	return searchRadius;
}

bool Memory::isRedundantFeature(const Histogram<153> &searchFeature, int featureId)
{
    int nMatches;
    PointCloud<Histogram<153> >::Ptr points(new PointCloud<Histogram<153> >);

    // no features in memory
    if(featuresMetadata.size() == 0)
		return false;

    // try to reduce the search domain
    if(clusters.size() > 0){
    	PointCloud<Histogram<153> >::Ptr centroids(new PointCloud<Histogram<153> >);
    	unordered_map<int, int> clustersIdxCorrespondence;

   	 	for(unordered_map<int, Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it) {
        	centroids->push_back(it->second.getCentroid());
			clustersIdxCorrespondence[centroids->size() - 1] = it->first;
    	}    

    	vector<int> idxNKNSearch;
    	vector<float> NKNSquaredDistance;

    	// find the k closests clusters centroids, k = 3
    	nMatches = searcher.KDtreeSPMatch(
			      searchFeature
			    , centroids, 3
			    , idxNKNSearch
			    , NKNSquaredDistance, false);    

    	if(nMatches == 0){
	    	throw(std::runtime_error("UNEXPECTED: sRedundantFeature(), No closest cluster found!"));
    	}

    	int clusterId;

    	Cluster *cluster;
    	unordered_set<int> *clusterPointIdxs;
    	
    	for(int i = 0; i < idxNKNSearch.size(); i++){
			clusterId = clustersIdxCorrespondence[idxNKNSearch.at(i)];

        	unordered_map<int, Cluster>::iterator it = clusters.find(clusterId);
        	if(it == clusters.end()){
	    	    cout << "CLUSTER ID: " << clusterId << endl;
	    	    throw(std::runtime_error("CLUSTER NOT FOUND IN MAP! CALL IN isRedundantFeature()."));
        	}

	        cluster = &clusters[clusterId];

		clusterPointIdxs = &cluster->getPoints();

		// get cluster points/features
		for(unordered_set<int>::iterator it = clusterPointIdxs->begin(); it != clusterPointIdxs->end(); ++it) {
			if(*it != featureId){
            		points->push_back(getFeatureMetadata(*it).getFeature());
			}
		}	
    	}
    // if no clusters are available
    }else{
	throw(std::runtime_error("NO CLUSTERS FOUND! CALL IN isRedundantFeature()."));
    }

    // search for the neareast feature
    vector<int> pointsIdxs;
    vector<float> pointsSquaredDistance;

    nMatches = searcher.KDtreeSPMatch(searchFeature
			    , points, 1
			    , pointsIdxs
			    , pointsSquaredDistance, false);

    //cout << pointsSquaredDistance.at(0) << endl;


    if(nMatches == 0){
		throw(std::runtime_error("UNEXPECTED NUMBER OF MATCHES! CALL IN isRedundantFeature()."));
    }

    // for debug purposes
    if(smallestDistance > pointsSquaredDistance.at(0))
		smallestDistance = pointsSquaredDistance.at(0);

    if(maxDistance < pointsSquaredDistance.at(0))
		maxDistance = pointsSquaredDistance.at(0);
    //-->

    // check distance to closest feature in memory
    if(pointsSquaredDistance.at(0) <= featureRedundancyTreshold){

		forgetFeature(featureId);
		numRedundantFeatures++;

		if(DEBUG) cout << "NEW FEATURE IS REDUNDANT! " << featureId << endl;

		return true;
    }

    return false;
}

void Memory::forgetRedundantFeatures(const vector<int> featuresIds)
{
    PointCloud<Histogram<153> *>::Ptr query(new PointCloud<Histogram<153> *>);
    PointCloud<Histogram<153> >::Ptr database(new PointCloud<Histogram<153> >);

	/*
	Histogram<153> *f = &getFeatureMetadata(featuresIds.at(0)).getFeature();

	double d = 0;
    	for(unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.begin(); it != featuresMetadata.end(); ++it) {
		d = searcher.euclideanDistance(it->second.getFeature(), *f);
		cout << d << endl;
    	}
	*/

    for(unordered_map<int, FeatureMetadata>::iterator it = featuresMetadata.begin(); it != featuresMetadata.end(); ++it) {
       	database->push_back(it->second.getFeature());
    }

    for(size_t i = 0; i < featuresIds.size(); i++)
    {
	query->push_back(&getFeatureMetadata(featuresIds.at(i)).getFeature());
    }

    vector<int> toForget;

    searcher.KDtreeCheckRedundancy(query
			    , database
			    , featureRedundancyTreshold
			    , toForget);

    for(size_t j = 0; j < toForget.size(); j++)
    {
	forgetFeature(featuresIds.at(toForget.at(j)));
	numRedundantFeatures++;

	//if(DEBUG) cout << "NEW FEATURE IS REDUNDANT! " << featuresIds.at(toForget.at(j)) << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void Memory::getPointCloud(PointCloud<Histogram<153> >::Ptr &cloud) {

    // cluster information is held in a map structure (a dictionary)
    // the map is iterated in order to build a vector holding all the
    // cluster centroids
    if(featuresMetadata.empty()) {
        return;
    }

    for(unordered_map<int, FeatureMetadata>::iterator feat=featuresMetadata.begin(); feat != featuresMetadata.end(); ++feat) {
        cloud->push_back(feat->second.getFeature());
        //clusterCentroidsMapper.push_back(feat->first);
    }
}

const int Memory::getClusterRandomPoint(const int &cluster) {

    // get a random point from the specified cluster
    return clusters[cluster].getRandomPoint();
}

const int Memory::getClusterRandomPoint(const int &cluster, int &pointIdx) {

    // get a random point from the specified cluster
    return clusters[cluster].getRandomPoint(pointIdx);
}
