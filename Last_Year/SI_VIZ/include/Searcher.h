
#ifndef SI_SEARCHER_H
#define SI_SEARCHER_H

#include <include.h>

/**
 *
 *
 *
 */
class Searcher {

	public:
		Searcher();

		~Searcher();

		int KDtreeSPMatch(PointCloud<SpinImage>::Ptr query
			, PointCloud<SpinImage>::Ptr database
			, int K
			, vector<int> &pointIdxNKNSearch
			, vector<float> &pointNKNSquaredDistance) const;

		int KDtreeSPMatch(const SpinImage &searchFeature
			, PointCloud<SpinImage>::Ptr database
			, int K
			, vector<int> &pointIdxNKNSearch
			, vector<float> &pointNKNSquaredDistance) const;

		/*
		void KDtreeSPMatch(PointCloud<SpinImage>::Ptr query
			, PointCloud<SpinImage>::Ptr database) const;
		*/

	private:
		//
};

#endif
