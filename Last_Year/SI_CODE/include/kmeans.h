#ifndef __KMEANS_H__
#define __KMEANS_H__

#include <map>
#include <string>
#include <vector>

#include <pcl/point_types.h> // for Histogram
#include <pcl/pcl_base.h>

#include "Memory.h"
#include "FeatureMetadata.h"

#include <tr1/unordered_map>

using std::tr1::unordered_map;

/**
 * \brief A custom implementation of Kmeans.
 *
 * The number of clusters is dynamic, i.e., there can be merge and split of clusters.
 */
class KMeans {

    public:
    int numOfClusters;
		/**
		* Constructor
		* @param[in] maxIters the maximum number of iterations in case convergence can not be achieved
		* @param[in] splitThresh The WSS (Within Sum of Squares) threshold to consider splitting a cluster
		* @param[in] mergeThresh The WSS (Within Sum of Squares) threshold to consider merging 2 clusters
		* @param[in] bssThresh The bss threshold
		* @param[in] wtssThresh The WSS/TSS ratio threshold
		* @param[in] wtssImproveThresh The improvement check threshold
		*/
		KMeans(int maxIters=10);	
		//KMeans(int maxIters=10, int maxCheckIters=2, float splitThresh=2.0f, float mergeThresh=1.0f, float bssThresh=1.0f,
		//		   float wtssThresh=0.2f, float wtssImproveThresh=0.1f);

		//KMeans(int maxIters=10, int numOfClusters=10);

		/**
		* Initialize KMEans by setting the internal reference (pointer)
		* to the memory structure
		*/

		void setNumberOfClusters(int number)
		{
			numOfClusters=number;
		}
		void init(Memory &memory);

		/**
		 * Update kmeans after forgeting a feature
		 */
		void update();

		/**
		* Assign features to clusters one at a time in a streaming fashion
		*
		* @param[in] featsIdx the keys (indices) of the features
		* @param[in] assignCluster flag controlling wether features should be assiged to a specific cluster or not
		*/
		void assign(std::vector<int> &featsIdx, int assignCluster=-1);

		/**
		* Assign features to clusters one at a time in a streaming fashion (called inside assign)
		*
		* @param[in] featsIdx the keys (indices) of the features
		* @param[in] assignCluster the identifier of the assignment cluster
		* @param[in] upCentroids flag controlling wether the centroids should be updated or not upon each feature assignment
		* @param[in] maxIts the number of times to run the assignment (defaults to 1)
		*/
		void assignFeatures(std::vector<int> &featsIdx, int &assignCluster, bool upCentroids=true, int maxIts=1);
    

		/**
		 * Temporary work around for fit. Used only in a specific situation!
		 * This should change!
		 *
		 */
		//void fit2();

		/**
		* Run the KMeans algorithm to perform the clustering until convergence or the
		* maximum number of iterations has been reached
		*
		* For computational performance KMeans runs only over the
		* features added at each batch of computed features
		*
		* @param[in] featsIdx the keys (indices) of the features
		*/
		//void fit();
		//void partialFit(vector<int> &featsIdx);

		/**
		* Assign the feature specified in featMetadata to the nearest cluster
		*
		* @param[in] centroids the cenroids of th cluters
		* @param[in] mapper mapper a structure that maps from the centroids vector indexes to the cluster identifiers
		* @param[in] revMapper a structure that maps from the cluster identifiers to the centroids vector indexes
		* @param[in] featMetadata the feature metadata (contains the feature to be assigned)
		* @param[in] featIdx the index of the feature in the memory vector
		* @param[in] k the number of nearest cluster centroids to consider for cluster assignment
		* @param[in] upCentroids if set to true the centroids will be updated after each new assignment
		* @param[in] assignCluster the identifier of the assignment cluster
		* @param[out] pCluster the previous cluster assignment (of the feature)
		* @param[out] nCluster the new cluster assignment (of the feature)
		*
		* Returns true if a new assignmnt took place, false otherwise

		* p.s. If an assignment cluster (assgnCluster) is specified, then the features are assigned to that cluster.
		* If no assignment cluster is specified (assgnCluster = -1), then the features are assigned to the nearest cluster
		*/
		bool assignToClusters(pcl::PointCloud<pcl::Histogram<153> >::Ptr &centroids, std::vector<int> &mapper,
								  unordered_map<int, int> &revMapper, FeatureMetadata &featMetadata, const int &featIdx, int k,
								  bool &upCentroids, int &assignCluster, int &pCluster, int &nCluster);

		/**
		* Find the nearest point amongst the given points to the specified point
		*
		* @param[in] points the points to search in for the closest point
		* @param[in] point the point to compare to the points
		* @param[in] kIndices returns the indices of the knn nearest centroids
		* @param[in] kSqrDistances returns the squared distances to the centroids
		* @param[in] knn the number of nearest centroids to search for
		*/
		void findNearestPoint(const pcl::PointCloud<pcl::Histogram<153> >::Ptr &points,
				const pcl::Histogram<153> &point, std::vector<int> &kIndices,
				std::vector<float> &kSqrDistances, int knn);

		/**
		* Recomputes all cluster centroids
		*/
		void recomputeCentroids();

		/**
		* Retrieve the cluster centroids stored in memory.
		* If no clustera are defined yet, a cluster is created and a random
		* point in memory is taken as its centroid
		*
		* @param[out] centroids the centroids of the clusters
		* @param[out] mapper a structure that maps from the centroids vector indexes to the cluster identifiers
		* @param[out] revMapper a structure that maps from the cluster identifiers to the centroids vector indexes
		*/
		void getCentroids(
			pcl::PointCloud<pcl::Histogram<153> >::Ptr &centroids, std::vector<int> &mapper, unordered_map<int, int> &revMapper);

		/**
		* Check the quality of the clusters
		*
		* @param[out] touchedClusters the list of clusters modified by the checking process
		*/
		void checkClustersValidity(vector<int> &touchedClusters);

		void checkValidity();
	
		/**
		* Perform a split operation on the specified cluster
		*
		* After the split, a fit must be performed in order to stabalize the newly created clusters
		*
		* @param[in] clusterId the cluster identifier
		* @param[out] touchedClusters the list of clusters modified by the checking process
		*/
		void splitCluster(int &clusterId, vector<int> &touchedClusters);

   		/**
		* Perform a merge operation between the specified clusters
		*
		* @param[in] lClusterId the first cluster identifier to merge
		* @param[in] rClusterId the second cluster identifier to merge
		* @param[out] touchedClusters the list of clusters modified by the checking process
		*/
		void mergeClusters(int &lClusterId, int &rClusterId, vector<int> &touchedClusters);
	
		/*
		*
		*/
		void cleanDegenerateCluster(int &clusterId);
	
	
		/**
		* Removes empty clusters from the clusters list
		*
		* @param[in] clusters metadata the list of candidate clusters to consider for removal
		*/
		void cleanEmptyClusters(vector<int> &clusters);
	
		/**
		* Check for if the specified cluster fits the conditions to be splitted
		*
		* @param[in] clusterMeta metadata concerning th cluster to be verified against the splitting conditions
		*/
		bool checkSplitConditions(WssBag &clusterMeta);

		/**
		* Check for if the specified cluster fits the conditions to be merged
		*
		* @param[in] clusterMeta metadata concerning th cluster to be verified against the merging conditions
		*/
		bool checkMergeConditions(WssBag &clusterMeta);
	
		/**
		*
		*/
		bool checkDegenerate(WssBag &clusterMeta);
		
		/**
		* Finds a suitable cluster to perform a merge operation with the specified cluster
		*
		* @param[in] clustersWss the list of candidate clusters
		* @param[in] currInd the index specifying the current cluster
		*
		* Returns the identifier of the cluster if one could be found, -1 otherwise
		*/
		int findClusterToMerge(vector<pair<int, WssBag> > &clustersWss, int &currInd);
    
    protected:

		//float lastWssClusterSplit;
	
		/**
		* The maximum number of iterations
		*/
		int maxIters;

//		int numOfClusters;
	
		/**
		* The maximum number of iterations for cluster validity check
		*/
		int maxCheckIters;

		/**
		* Pointer to the memory structure holding the features
		*/
		Memory *memory;

		/**
			Thresholds
		*/

		/**
		* The WSS/TSS ratio
		*
		* Holds the last WSS/TSS ratio computed to check cluster quality
		*/
		//float wtssRatio;

		/**
		* The WSS (Within Sum of Squares) threshold to consider splitting a cluster
		*
		* The cluster will be consider for spliting if its WSS measure is above this value
		*
		* y default it is innitialized to 2.0f
		*/
		//float splitThresh;

		/**
		* The WSS (Within Sum of Squares) threshold to consider merging 2 clusters
		*
		* The cluster will be consider for merging if its WSS measure is below this value
		*
		* By default it is innitialized to 1.0f
		*/
		//float mergeThresh;

		/**
		* The bss threshold
		*
		* Used to check clustering quality. If the BSS measure between 2 clusters
		* falls below this threshold, then the cluster will be considered for merging
		* (in case their individual WSS measures are also below the 'mergeThresh' value)
		*
		* By default it is innitialized to 1.0f
		*/
		//float bssThresh;

		/**
		* The WSS/TSS ratio threshold
		*
		* Used to check clustering quality. A value close to or below 0.2
		* means good clustering quality.  A value above is an indication
		* of non-optimum clustering quality
		*
		* By default it is innitialized to 0.2f
		*/    
		//float wtssThresh;

		/**
		* The improvement check threshold
		*
		* If only a small improvement would be achieved, then no attempt to
		* improve clustering quality is performed, and is delayed until a bigger
		* improvement can be achieved
		*
		* By default it is innitialized to 0.1f
		*/
		//float wtssImproveThresh;
	
		// cluster wss value suitable for a split operation
		float wssThreshForSplit;
	
		// the minimum size a cluster must have in order
		// to be considered for splitting
		int minSplitClusterSize;
	
		// the maximum size a cluster must have
		// after getting this size the cluster can be considered for splitting
		int maxSplitClusterSize;
	
		// cluster wss value suitable for a merge operation
		float wssThreshForMerge;

		// bss between clusters suitable for a merge operation
		float bssThreshForMerge;

		// the number of elements that specify a cluster as being degenerate (too few elements)
		int degenerateClusterSize;
	
    public:
    
		/**
		 * Update the value of wtssRatio
		 *
		 * @param[in] ratio the value to perform the update
		 */
		/*void updateWtssRatio(float &ratio){
			wtssRatio = ratio;
		}*/
    
};

#endif  // __KMEANS_H__
