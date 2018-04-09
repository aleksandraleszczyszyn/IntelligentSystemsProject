#ifndef SI_SEARCHER_H
#define SI_SEARCHER_H

#include <include.h>

/**
 * \brief A class with functions to search for features in a given search domain.
 * It also provides functions for the calculation of the euclidean distance for vectors.
 *
 *
 */
class Searcher {

	public:
		Searcher();

		~Searcher();
	
		/**
		 * IMPORTANT: Don't use this function!
 		 * Searches for the K closest Neighbor points of a given point using KdTreeFLANN.
		 *
		 * @param[in] query 							set of points to search
		 * @param[in] database	 						search domain
		 * @param[in] maximumReturnedNeighbors			maximum number of points to return from the 
		 *												"solutions" set
		 * @param[out] neighborPointsIdxs				ids of the returned "solutions" (i.e., points 
		 *												in the search domain), maximumReturnedNeighbors at most.
		 * @param[out] neighborPointsSquaredDistances	squared distances between the search point 
		 *												and the retruned Neighbors (solutions)
		 * 
		 *
		 *  
		 * returns the number of matches
		 */
		int KDtreeSPMatch(
			  const PointCloud<SpinImage>::Ptr query
			, const PointCloud<SpinImage>::Ptr database
			, int maximumReturnedNeighbors
			, vector<int> &neighborPointsIdxs
			, vector<float> &neighborPointsSquaredDistances) const;

		/**
 		 * Searches for the K closest Neighbor points of a given point using KdTreeFLANN.
		 *
		 * @param[in] searchFeature 					point to search
		 * @param[in] database	 						search domain
		 * @param[in] maximumReturnedNeighbors			maximum number of points to return from the 
		 *												"solutions" set
		 * @param[out] neighborPointsIdxs				ids of the returned "solutions" (i.e., points 
		 *												in the search domain)
		 * @param[out] neighborPointsSquaredDistances	squared distances between the search point 
		 *												and the retruned Neighbors (solutions)
		 * 
		 *  
		 * returns the number of matches
		 */
		int KDtreeSPMatch(
			  const SpinImage &searchFeature
			, const PointCloud<SpinImage>::Ptr database
			, int maximumReturnedNeighbors
			, vector<int> &neighborPointsIdxs
			, vector<float> &neighborPointsSquaredDistances, bool sorted) const;

		/**
 		 * Searches for the Neighbor points of a given point in a radius, using KdTreeFLANN.
		 *
		 * @param[in] searchFeature 					point to search
		 * @param[in] database	 						search domain
		 * @param[in] searchRadius	 					search radius
		 * @param[in] maximumReturnedNeighbors			maximum number of points to return from the 
		 *												"solutions" set
		 * @param[out] neighborPointsIdxs				ids of the returned "solutions" (i.e., points 
		 *												in the search domain)
		 * @param[out] neighborPointsSquaredDistances	squared distances between the search point 
		 *												and the retruned Neighbors (solutions)
		 *  
		 * returns the number of matches
		 */
		int KDtreeRadiusSearch(
			  const SpinImage &searchFeature
			, const PointCloud<SpinImage>::Ptr database
			, double searchRadius
			, int maximumReturnedNeighbors
			, vector<int> &neighborPointsIdxs
			, vector<float> &neighborPointsSquaredDistances) const;

		void KDtreeCheckRedundancy(
			  const PointCloud<SpinImage *>::Ptr query
			, const PointCloud<SpinImage>::Ptr database
			, double redundancyThreshold
			, vector<int> &toForget) const;

		/**
 		 * Calculates the euclidean distance between two vectors, vectorA and vectorB.
		 * The vectors are assumed to be normalized.
		 *
		 * @param[in] vectorA
		 * @param[in] vectorB
		 * @param[in] size the size of the vectors
		 *  
		 * returns the euclidean distance between the two vectors
		 */
		double euclideanDistance(const double* vectorA, const double* vectorB, size_t size) const;

		/**
 		 * Calculates the euclidean distance between two vectors, vectorA and vectorB.
		 * The vectors are assumed to be normalized.
		 *  
		 * @param[in] vectorA
		 * @param[in] vectorB
		 *
		 * returns the euclidean distance between the two vectors
		 */
		double euclideanDistance(const vector<double>& vectorA, const vector<double>& vectorB) const;

		/**
 		 * Calculates the euclidean distance between two spin images, A and B.
		 * The vectors are assumed to be normalized.
		 *  
		 * @param[in] A
		 * @param[in] B
		 *
		 * returns the euclidean distance between the two spin images
		 */
		double euclideanDistance(const SpinImage& A, const SpinImage& B) const;
};

#endif
