#include <math.h>
#include <algorithm>
#include "CategoryRecognizer.h"

using namespace pcl;
using namespace std;

CategoryRecognizer::CategoryRecognizer(Memory& memory, bool debugMode) 
	: memory(memory), DEBUG(debugMode)
{}

CategoryRecognizer::~CategoryRecognizer()
{}

std::vector<double>* CategoryRecognizer::getViewHistogram(std::vector<int>& featsIdx, vector<int> &redundancyFeatsIdx)
{
    unordered_map<int, double> histogram;
    vector<int> clusterIds;
    double squaredSum = 0;
    float localWeight = 0;

    unordered_map<int, Cluster> clusters = memory.getClusters();

    // histogram has a dimension for every cluster
    for(unordered_map<int, Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it) {
		histogram[it->first] = 0.0;
		clusterIds.push_back(it->first);
    }

    std::sort(clusterIds.begin(), clusterIds.end());

    const FeatureMetadata *featureInfo;

    // compute un-normalized histogram
    for(size_t i = 0; i < featsIdx.size(); i++){
		featureInfo = &memory.getFeatureMetadata(featsIdx.at(i));
		histogram[featureInfo->getCluster()] += featureInfo->getLocalWeight();
    }    

    for(size_t i = 0; i < redundancyFeatsIdx.size(); i++){
		featureInfo = memory.getFeatureMetadata2(redundancyFeatsIdx.at(i));

		if(featureInfo == NULL){
			cout << "FEATURE: " << redundancyFeatsIdx.at(i) 
				 << " USED FOR REDUNDANCY WAS FORGOTTEN. IGNORING FEATURE IN VIEW HISTOGRAM!" 
				 << endl;
			continue;
		}

		histogram[featureInfo->getCluster()] += featureInfo->getLocalWeight();
    }    

    // normalized view histogram
    std::vector<double>* H = new std::vector<double>();

    for(size_t i = 0; i < clusterIds.size(); i++){
	localWeight = histogram[clusterIds[i]];
	H->push_back(localWeight);

	squaredSum += localWeight * localWeight;
    }

    squaredSum = std::sqrt(squaredSum);

    for(size_t i = 0; i < H->size(); i++){
	H->at(i) /= squaredSum;
    }

	if(DEBUG){
		cout << "VIEW HISTOGRAM: ";
		for(size_t i = 0; i < H->size(); i++){
			cout << H->at(i) << " ";
		}
		cout << endl;		
	}

    return H;
}

std::vector<double>* CategoryRecognizer::getViewHistogram(std::vector<int>& featsIdx)
{
    unordered_map<int, double> histogram;
    vector<int> clusterIds;
    double squaredSum = 0;
    float localWeight = 0;

    unordered_map<int, Cluster> clusters = memory.getClusters();

    // histogram has a dimension for every cluster
    for(unordered_map<int, Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it) {
		histogram[it->first] = 0.0;
		clusterIds.push_back(it->first);
    }

    std::sort(clusterIds.begin(), clusterIds.end());

    const FeatureMetadata *featureInfo;

    // compute un-normalized histogram
    for(size_t i = 0; i < featsIdx.size(); i++){
		featureInfo = &memory.getFeatureMetadata(featsIdx.at(i));
		histogram[featureInfo->getCluster()] += featureInfo->getLocalWeight();
    }    

    // normalized view histogram
    std::vector<double>* H = new std::vector<double>();

    for(size_t i = 0; i < clusterIds.size(); i++){
		localWeight = histogram[clusterIds[i]];
		H->push_back(localWeight);

		squaredSum += localWeight * localWeight;
    }

    squaredSum = std::sqrt(squaredSum);

    for(size_t i = 0; i < H->size(); i++){
		H->at(i) /= squaredSum;
    }

	if(DEBUG){
		cout << "VIEW HISTOGRAM: ";
		for(size_t i = 0; i < H->size(); i++){
			cout << H->at(i) << " ";
		}
		cout << endl;		
	}

    return H;
}

int CategoryRecognizer::getViewCategory(std::vector<double>& viewHistogram)
{
    unordered_map<int, double> histogram;
    vector<int> clusterIds;
    vector<CategoryViewDistance> categoryViewDistance;
    std::vector<double> categoryHistogram;

    double squaredSum = 0;
    float localWeight = 0;

    Searcher searcher;
    double distance;

    unordered_map<int, Cluster> clusters = memory.getClusters(); 
    unordered_map<int, Category> categories = memory.getCategories();

    // get cluster ids
    for(unordered_map<int, Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it) {
		histogram[it->first] = 0;
		clusterIds.push_back(it->first);
    }

    // sort cluster ids
    std::sort(clusterIds.begin(), clusterIds.end());

    // get categories histograms
    for(unordered_map<int, Category>::iterator it = categories.begin(); it != categories.end(); ++it) {
        unordered_set<int>& points = it->second.getPoints();

		for(unordered_set<int>::iterator ptIdx = points.begin(); ptIdx != points.end(); ++ptIdx) {
			FeatureMetadata& featureInfo = memory.getFeatureMetadata(*ptIdx);
			histogram[featureInfo.getCluster()] += featureInfo.getLocalWeight();
		}

		categoryHistogram.erase(categoryHistogram.begin(), categoryHistogram.end());

        for(size_t i = 0; i < clusterIds.size(); i++){
			localWeight = histogram[clusterIds[i]];

			categoryHistogram.push_back(localWeight);

			squaredSum += localWeight * localWeight;
			histogram[clusterIds[i]] = 0;
        }

		squaredSum = std::sqrt(squaredSum);

        for(size_t i = 0; i < categoryHistogram.size(); i++){
			categoryHistogram.at(i) /= squaredSum;
        }
		
		if(DEBUG){
			cout << "CATEGORY " << it->second.getName() << " Id: " << it->first << " HISTOGRAM ";
			for(size_t i = 0; i < categoryHistogram.size(); i++){
				cout << categoryHistogram.at(i) << " ";
			}
			cout << endl;			
		}

		// get distance between the current category and the view
		distance = searcher.euclideanDistance(categoryHistogram, viewHistogram);
		categoryViewDistance.push_back(CategoryViewDistance(it->first, distance));
    }

    // sort the distances
    std::sort(categoryViewDistance.begin(), categoryViewDistance.end());

    // print distance for debug
    if(DEBUG){
		cout << "CATEGORY ID | VIEW : DISTANCE" << endl;
		for(size_t i = 0; i < categoryViewDistance.size(); i++){
		cout << categoryViewDistance.at(i).categoryId << " : " 
			 << categoryViewDistance.at(i).distance << endl;
		}  
    }

    // return the id of the category nearest to the view
    return categoryViewDistance.at(0).categoryId;
}

void CategoryRecognizer::assignViewToCategory(std::vector<int>& featsIdx, int categoryId)
{
    memory.assignViewToCategory(featsIdx, categoryId);
}
