#include <stdlib.h>
#include "Cluster.h"

using namespace std;
using namespace pcl;
    
Cluster::Cluster(int identifier, Histogram<153> &center)
    : identifier(identifier) {

    setCentroid(center);
}

void Cluster::setCentroid(Histogram<153> &center) {
    copy(center.histogram, center.histogram + 153, centroid.histogram);
    copy(center.histogram, center.histogram + 153, sumFeatures.histogram);
    sumWeight = 1;
}

void Cluster::addPoint(int &idx) {
    points.insert(idx);
}

void Cluster::removePoint(int &idx) {
    points.erase(idx);
}

void Cluster::clearPoints() {
    points.clear();
}

void Cluster::getPoints(vector<int> &pointsIdx) const {
    pointsIdx.resize(points.size());
    copy(points.begin(), points.end(), pointsIdx.begin());
}

const int Cluster::getRandomPoint() {
    int pointIdx = -1;
    
    if (size() > 0) {
        // generate a andom number in the range 0-set.size()
        int randIdx = rand() % points.size();
        //set<int>::iterator it = points.begin();
	unordered_set<int>::iterator it = points.begin();
        advance(it, randIdx);
        pointIdx = *it;
    }

    return pointIdx;
}

const int Cluster::getRandomPoint(int &pointIdx) {
    int nPointIdx = pointIdx;

    // get a random number making sure we do not get a value
    // equal to the one specified by pointIdx
    while(nPointIdx == pointIdx) {
        nPointIdx = getRandomPoint();
    }

    return nPointIdx;
}

float &Cluster::addToSumWeight(const float &weight) {
    sumWeight += weight;
    return sumWeight;
}

float &Cluster::subtractFromSumWeight(const float &weight) {
    sumWeight -= weight;
    return sumWeight;
}

Histogram<153> &Cluster::addToSumFeatures(Histogram<153> &feat, const float &featWeight) {
    for(int i=0; i < 153; ++i) {
        sumFeatures.histogram[i] += feat.histogram[i] * featWeight;
    }
    return sumFeatures;
}

Histogram<153> &Cluster::removeFromSumFeatures(Histogram<153> &feat, const float &featWeight) {
    for(int i=0; i < 153; ++i) {
        sumFeatures.histogram[i] -= feat.histogram[i] * featWeight;
    }
    return sumFeatures;
}

void Cluster::recomputeCentroid() {
    for(int i=0; i < 153; ++i) {
        centroid.histogram[i] = sumFeatures.histogram[i] / sumWeight;
    }
}

void Cluster::resetClusterTotals() {
    resetSumWeight();
    resetSumFeatures();
}
