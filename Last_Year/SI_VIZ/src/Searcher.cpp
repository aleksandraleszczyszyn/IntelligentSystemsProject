

#include <Searcher.h>

Searcher::Searcher()
{
	//TODO
}

Searcher::~Searcher()
{
	//TODO
}

int Searcher::KDtreeSPMatch(PointCloud<SpinImage>::Ptr query
	, PointCloud<SpinImage>::Ptr database
	, int k
	, vector<int> &pointIdxNKNSearch
	, vector<float> &pointNKNSquaredDistance) const
{
    //declare variables for Kd tree search
    KdTreeFLANN<SpinImage > kdtree;
    //int match = 0;
    //int temp_dist2NN;
    SpinImage searchFeature;

    //search for 2NN and check for ratio threshold
    //int K = 2;
    /*
    vector<int> pointIdxNKNSearch(K);
    vector<float> pointNKNSquaredDistance(K);
    */

    //initilization for kd-tree
    kdtree.setInputCloud(database);

    //compare each query point with each scene point
    int numFoundMatches = 0;

    for (int i = 0; i < query->size(); i++)
    {
        searchFeature = query->points.at(i);

        if(isnan(searchFeature.histogram[0]))
	{
	    //cout << c << " is NAN. Skipping!" << endl;
            continue;
	}

        numFoundMatches += kdtree.nearestKSearch(searchFeature
						, k
						, pointIdxNKNSearch
						, pointNKNSquaredDistance);

	/*
        if(numFoundMatches > 0)
        {
            temp_dist2NN = pointNKNSquaredDistance.at(0);
	    cout << "NN distance: " << temp_dist2NN << endl;
        }


         if (temp_dist2NN < 0.25f){
              match++;
         }

        cout << "si " << c << " in query matches si "<< pointIdxNKNSearch.at(0) << " in db." << endl;
	*/
    }

    //show status
    //cout << "total matchs: " << match << endl;
    return numFoundMatches;
}

int Searcher::KDtreeSPMatch(const SpinImage &searchFeature
	, PointCloud<SpinImage>::Ptr database
	, int k
	, vector<int> &pointIdxNKNSearch
	, vector<float> &pointNKNSquaredDistance) const
{
    KdTreeFLANN<SpinImage > kdtree;

    kdtree.setInputCloud(database);

    int N = 0;

    if(!isnan(searchFeature.histogram[0]))
    {
        N = kdtree.nearestKSearch(searchFeature
				, k
				, pointIdxNKNSearch
				, pointNKNSquaredDistance);
    }

    return N;
}
