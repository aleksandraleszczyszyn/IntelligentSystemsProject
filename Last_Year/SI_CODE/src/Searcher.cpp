
#include <Searcher.h>

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

Searcher::Searcher()
{}

Searcher::~Searcher()
{}

double Searcher::euclideanDistance(const SpinImage& A, const SpinImage& B) const
{
    double distance = 0.0;
    double dx;

    double squaredSumA = 0.0;
    double squaredSumB = 0.0;

    for (size_t i = 0; i < 153; i++){
	squaredSumA += A.histogram[i] * A.histogram[i];
	squaredSumB += B.histogram[i] * B.histogram[i];
    }

    squaredSumA = std::sqrt(squaredSumA);
    squaredSumB = std::sqrt(squaredSumB);

    for (size_t i = 0; i < 153; i++){
        dx = A.histogram[i] / squaredSumA - B.histogram[i] / squaredSumB;
        distance += (dx * dx);
    }

    return distance > 0.0 ? sqrt(distance) : 0.0;
}

double Searcher::euclideanDistance(const vector<double>& vectorA, const vector<double>& vectorB) const
{
    double distance = 0.0;
    double dx;

    if(vectorA.size() != vectorB.size()){
	throw(
	std::runtime_error("THE VECTORS MUST HAVE THE SAME SIZE WHEN COMPUTING THE EUCLIDEAN DISTANCE.")
	);
    }

    for (size_t i = 0; i < vectorA.size(); i++){
        dx = vectorA.at(i) - vectorB.at(i);
        distance += (dx * dx);
    }

    return distance > 0.0 ? sqrt(distance) : 0.0;
}

double Searcher::euclideanDistance(const double* vectorA, const double* vectorB, size_t size) const
{
    double distance = 0.0;
    double dx;

    if(vectorA == NULL || vectorB == NULL)
	throw(std::runtime_error("vectorA or vectorB IS NULL."));

    for (size_t i = 0; i < size; i++){
        dx = *(vectorA + i) - *(vectorB + i);
        distance += (dx * dx);
    }

    return distance > 0.0 ? sqrt(distance) : 0.0;
}

int Searcher::KDtreeSPMatch(
	  const PointCloud<SpinImage>::Ptr query
	, const PointCloud<SpinImage>::Ptr database
	, int maximumReturnedNeighbors
	, vector<int> &neighborPointsIdxs
	, vector<float> &neighborPointsSquaredDistances) const
{
    KdTreeFLANN<SpinImage > kdtree;
    SpinImage searchFeature;

    kdtree.setInputCloud(database);

    int numFoundMatches = 0;

    // compare each query point with each scene point
    for (int i = 0; i < query->size(); i++)
    {
        searchFeature = query->points.at(i);

        if(isnan(searchFeature.histogram[0]))
	    throw(std::runtime_error("KDtreeSPMatch() NaN."));

        numFoundMatches += kdtree.nearestKSearch(searchFeature
						, maximumReturnedNeighbors
						, neighborPointsIdxs
						, neighborPointsSquaredDistances);

    }

    return numFoundMatches;
}

void Searcher::KDtreeCheckRedundancy(
	  const PointCloud<SpinImage *>::Ptr query
	, const PointCloud<SpinImage>::Ptr database
	, double redundancyThreshold
	, vector<int> &toForget) const
{
    KdTreeFLANN<SpinImage > kdtree;
    //SpinImage searchFeature;
    vector<int> neighborPointsIds;
    vector<float> neighborPointsSquaredDistances;

    kdtree.setInputCloud(database);

    int numFoundMatches = 0;

    // compare each query point with each scene point
    for (int i = 0; i < query->size(); i++)
    {
        //searchFeature = query->points.at(i);
	/*
        if(isnan(searchFeature.histogram[0]))
	    throw(std::runtime_error("KDtreeSPMatch() NaN."));
	*/

        numFoundMatches = kdtree.nearestKSearch(*query->points.at(i)
						, 2
						, neighborPointsIds
						, neighborPointsSquaredDistances);

	if(neighborPointsSquaredDistances.at(0) != 0){
		if(neighborPointsSquaredDistances.at(0) <= redundancyThreshold)
			toForget.push_back(i);
	}else{
		//cout << "D: " << neighborPointsSquaredDistances.at(1) << endl;
		if(neighborPointsSquaredDistances.at(1) <= redundancyThreshold)
			toForget.push_back(i);
	}

	neighborPointsIds.erase(neighborPointsIds.begin(), neighborPointsIds.end());
	neighborPointsSquaredDistances.erase(neighborPointsSquaredDistances.begin(), neighborPointsSquaredDistances.end());
    }
}

int Searcher::KDtreeSPMatch(
	  const SpinImage &searchFeature
	, const PointCloud<SpinImage>::Ptr database
	, int maximumReturnedNeighbors
	, vector<int> &neighborPointsIdxs
	, vector<float> &neighborPointsSquaredDistances, bool sorted = true) const
{
    KdTreeFLANN<SpinImage > kdtree(sorted);
    kdtree.setInputCloud(database);

    int N = 0;

    if(isnan(searchFeature.histogram[0]))
	throw(std::runtime_error("KDtreeSPMatch() NaN."));

    N = kdtree.nearestKSearch(searchFeature
			    , maximumReturnedNeighbors
			    , neighborPointsIdxs
			    , neighborPointsSquaredDistances);

    return N;
}

int Searcher::KDtreeRadiusSearch(
	  const SpinImage &searchFeature
	, const PointCloud<SpinImage>::Ptr database
	, double searchRadius
	, int maximumReturnedNeighbors
	, vector<int> &neighborPointsIdxs
	, vector<float> &neighborPointsSquaredDistances) const
{
    KdTreeFLANN<SpinImage > kdtree(false);
    kdtree.setInputCloud(database);

    int N = 0;

    if(isnan(searchFeature.histogram[0]))
	throw(std::runtime_error("KDtreeRadiusSearch() NaN."));

    N = kdtree.radiusSearch( searchFeature
			   , searchRadius
			   , neighborPointsIdxs
			   , neighborPointsSquaredDistances
			   , maximumReturnedNeighbors);

    return N;
}
