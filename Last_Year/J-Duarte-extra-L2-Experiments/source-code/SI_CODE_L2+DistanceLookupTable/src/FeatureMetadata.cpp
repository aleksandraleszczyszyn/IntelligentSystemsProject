#include "FeatureMetadata.h"

using namespace pcl;

FeatureMetadata::FeatureMetadata(Histogram<153> &feature, float lWeight, float gWeight,
    int count, int cluster, float dClusterCentroid, int category)

    : feature(feature), lWeight(lWeight), gWeight(gWeight), count(count),
    cluster(cluster), dClusterCentroid(dClusterCentroid), category(category) {
}
