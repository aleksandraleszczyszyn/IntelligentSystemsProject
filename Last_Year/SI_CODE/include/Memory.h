#ifndef __MEMORY_H__
#define __MEMORY_H__

#include <map>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/common/point_tests.h>

#include "FeatureMetadata.h"
#include "Cluster.h"
#include "Searcher.h"
#include "Category.h"
#include "SeqGenerator.h"
#include "WssBag.h"
#include "BssBag.h"

#include <stdexcept>

#include <tr1/unordered_map>

using std::tr1::unordered_map;

class KMeans;

/**
 * \brief Main structure of the application. Holds the applications' memory.
 *
 *
 */
class Memory {
    
    public:

    	Memory();

    	~Memory();

		/**
      	* Updates the cenroids of the clusters after feature assignment
		*
       	* uses the formula : mean(k) = mean(k-1) + (feat - mean(k-1))/k (for new assignments)
      	* mean(k) is the new centroid to be computed
     	* mean(k-1) is the previous centroid (before the new featue assignment)
       	* feat is the newly assigned feature to the cluster
      	* k is the number of elements in the cluster after the new assignment
		*
      	* @param[in] pClusterId the cluster identifier of the previous cluster assignment
    	* @param[in] nClusterId the cluster identifier of the new cluster assignment
        * @param[in] featIdx the feature map key
      	*/
        void updateCentroids(int pClusterId, int nClusterId, int featIdx);

        /**
     	* Synchronizes cluster assignment for features
    	*
       	* Removes the feature from the list of points from the
    	* previous cluster assignment if one exists and adds the feature to
    	* the list of points of the new assigned cluster
		*
     	* @param[in] pClusterId the cluster identifier of the previous cluster assignment
      	* @param[in] nClusterId the cluster identifier of the new cluster assignment
     	* @param[in] featIdx the feature map key
      	*/    
        void syncClusterAssignment(int pClusterId, int nClusterId, int featIdx);

        /**
		* Adds a feature (point) to memory- It also adds the corresponding metadata
      	* to the feature metadata structure.
		*
      	* @param[in] feat the feature to be added to the memory
		*
		* Returns the identifier of the created feature.
     	*/
        char addFeature(pcl::Histogram<153> &feat, int &featureId);

		/**
		 * The same as in getFeatureMetadata but we can have a NULL return.
		 *
		 */
    	const FeatureMetadata* getFeatureMetadata2(const int &key);

        /**
		* Returns the feature metadata structure stored at the specified map key
		*
		* @param[in] key the key
		*/
        FeatureMetadata &getFeatureMetadata(const int &key);
    
		/**
		* Returns the feature stored at the specified map key
		*
		* @param[in] key the key
		*/ 
		pcl::Histogram<153> &getFeature(const int &key);   

		/**
		* Retrieve the cluster centroids stored in memory.
		*
		* @param[out] centroids the centroids of the clusters
		* @param[out] mapper a structure that maps from the centroids vector indexes to the cluster identifiers
		* @param[out] revMapper a structure that maps from the cluster identifiers to the centroids vector indexes
		*/
		void getCentroids(
			pcl::PointCloud<pcl::Histogram<153> >::Ptr &centroids, std::vector<int> &mapper, unordered_map<int, int> &revMapper);

		/**
		*
		*/
		void getClusterPointsAsCloud(PointCloud<Histogram<153> >::Ptr &points, vector<int> &pointsMapper, int &clusterId);
			
		/**
		* Get the centroid of a specific cluster
		*
		* @param[in] clusterId the cluster identifier
		*/
		pcl::Histogram<153> &getClusterCentroid(int &clusterId);
		
		/**
		* Gets the size of a specific cluster.
		*
		* @param[in] clusterId the cluster identifier
		*/
		int getClusterSize(int &clusterId);
		
		/**
		* Create a new cluster
		*
		* @param[in] centroid the centroid of the cluster
		*
		* Returns the identifier of the created cluster
		*/
		int addCluster(pcl::Histogram<153> &centroid);

		/**
		* Delete a cluster
		*
		* @param[in] clusterId the identifier of the cluster to delete
		*/
		void deleteCluster(int &clusterId);

		/**
		* Add a feature to the cluster
		*
		* @param[in] clusterId the identifier of the cluster
		* @param[in] featIdx the index of the feature (in main memory) to add to the cluster
		*/
		void addPointToCluster(int &clusterId, int &featIdx);
		
		/**
		* Remove a feature from the cluster
		*
		* @param[in] clusterId the identifier of the cluster
		* @param[in] featIdx the index of the feature (in main memory) to remove from the cluster
		*/
		void removePointFromCluster(int &clusterId, int &featIdx);

		/**
		* Sets all features assigned to the specified cluster to -1 (in the cluster identifier field)
		* in order to mark them as unassigned
		*
		* @param[in] clusterId the identifier of the cluster
		*/
		void unAssignAllPointsFromCluster(int &clusterId);
		
		/**
		* Sets the centroid of a cluster
		*
		* @param[in] clusterId the identifier of the cluster
		* @param[in] featIdx the index of the feature (in main memory) to be used as the cluster centroid
		*/
		void setClusterCentroid(int &clusterId, int &featIdx);

		/**
		* Resets the running totals of the specified cluster; sumWeight and sumFeatures
		*/
		void resetClusterTotals(int &clusterId);

		/**
		* Returns the cluster identifier for the biggest cluster of the 2 clusters specified
		*
		* @param[in] lClusterId the identifier of the first cluster to be compared in terms of size
		* @param[in] rClusterId the identifier of the second cluster to be compared in terms of size
		*/
		int &getBiggerCluster(int &lClusterId, int &rClusterId);

		/**
		* Return the list of features assigned to the specified cluster
		*
		* @param[in] clusterId the identifier of the cluster to get the list of assigned features
		* @param[out] pointsIdx the list of features
		*/
		void getClusterPoints(int &clusterId, std::vector<int> &pointsIdx);
		
		/**
		* Compute the distance between two clusters
		*
		* @param[in] lCluster the identifier of the first cluster
		* @param[in] rCluster the identifier of the second cluster
		*
		*/
		float bssBetwenClusters(int& lCluster, int& rCluster);

		/**
		* Compute BSS (Between cluster Sum of Squares) measure
		*
		* @param[out] clustersBss vector holding the 'partial bss' computed for every cluster
		*
		* Returns the total bss
		*/
		float bss(unordered_map<int, BssBag> &clustersBss);
	
		/**
		* Compute WSS (Whithin cluster Sum of Squares) measure
		*
		* @param[out] clustersWss vector holding the wss computed for every cluster, sorted by descending order (bigger wss values first)
		*
		* Returns the total wss
		*/
		float wss(std::vector<std::pair<int, WssBag> > &clustersWss);

		/*Category*/
	   
		/**
		* Return category identifier for specified categoy name
		*
		* @param[in] cat the category name to return the identifier
		*/
		int getCategoryId(const std::string &categoryName);
	 
		/**
		* Create a new category
		*
		* @param[in] name the name of the category
		*
		* Returns the identifier of the created category
		*/
		int addCategory(std::string categoryName);    

		/**
		* Add a feature to the category
		*
		* @param[in] catId the identifier of the category
		* @param[in] featIdx the index of the feature (in main memory) to add to the category
		*/
		void addPointToCategory(int catId, int featIdx);
		
		/**
		* Remove a feature from the category
		*
		* @param[in] catId the identifier of the category
		* @param[in] featIdx the index of the feature (in main memory) to remove from the category
		*/ 
		void removePointFromCategory(int &catId, int &featIdx);

		/**
		* Prints information to the console about the categories stored in memory.
		*/ 
		void printCategoriesInfo();

		const std::string& getCategoryName(int categoryId);

		/**
		* Assings the specified features ids to the specified category.
.		* @param[in] featsIdx features ids to assign
		* @param[in] categoryId the identifier of the category to assign the features to
		*/ 
		void assignViewToCategory(std::vector<int>& featsIdx, int categoryId);

		/**
		* Increments the True Positives count for the specified category
		*
		* @param[in] categoryId the category identifier
		*/
		void addTPToCategory(int &categoryId);

		/**
		* Increments the False Positives count for the specified category
		*
		* @param[in] categoryId the category identifier
		*/
		void addFPToCategory(int &categoryId);

		/**
		* Increments the False Negatives count for the specified category
		*
		* @param[in] categoryId the category identifier
		*/
		void addFNToCategory(int &categoryId);

		/*End Category*/

		/*View*/

		/**
		 * Don use this function! Use CategoryRecognizer instead!
		 * 
		 */
		std::vector<double>* getViewHistogram(std::vector<int>& featsIdx, vector<int> &redundancyFeatsIdx);

		/*End View*/

		void getPointCloud(pcl::PointCloud<pcl::Histogram<153> >::Ptr &cloud);

		/**
		* Get a random point from the specified cluster identifier
		*
		* @param[in] cluster the cluster identifier
		*
		* Returns the index to the feature
		*/
		const int getClusterRandomPoint(const int &cluster);

		/**
		* Get a random point from the specified cluster identifier. The same as
		* getClusterRandomPoint(int) except that the value specified by the
		* parameter pointIdx should not be repeated
		*
		* @param[in] cluster the cluster identifier
		* @param[in] pointIdx the cluster identifier
		*
		* Returns the index to the feature
		*/
		const int getClusterRandomPoint(const int &cluster, int &pointIdx);
	 
		/**
		 * Don't use this function! Use CategoryRecognizer instead!
		 * 
		 */
		int getViewCategory(std::vector<double>& viewHistogram);

		/**
		 * Checks if a given feature, new seen feature not yet in memory, is redundant.
		 * Redundancy follows from the use of a treshold min distance.
		 * If the given feature is redundant, the id of the feature in memory making it
		 * redundant is returned.
		 * New seen/added features, not yet assign to a cluster, are also used to look for redundancy.
		 *
		 * searchFeature	feature to look for redundancy
		 * featureId		the id of the feature in memory making the given feature redundant
		 *					if the feature is found redundant
		 *
		 * Note: It is assumed that the feature to look for redundancy is not yet in memory!
		 *		 This will became an issue if the architecture changes!
		 *
		 * returns true if redundant or false otherwise.
		 */
		bool isRedundantFeature(const Histogram<153> &searchFeature, int featureId);

		/**
		 * Deletes/removes a given feature from memory. No redistribution of weights is made in this function!
		 * 
		 * @param[in] featureId the id of the feature to delete/forget.
		 */
		void removeFeatureFromMemory(int featureId);

		void setMemoryDecayFactor(float aMemoryDecayFactor){
			memoryDecayFactor = aMemoryDecayFactor;
		}

		float getMemoryDecayFactor(){
			return memoryDecayFactor;
		}

		void setMemoryDecayTreshold(float aMemoryDecayTreshold){
			memoryDecayTreshold = aMemoryDecayTreshold;
		}

		float getMemoryDecayTreshold(){
			return memoryDecayTreshold;
		}

		long long getNumForgottenFeatures(){
			return numForgottenFeatures;
		}

		long long getNumSeenFeatures(){
			return numSeenFeatures;
		}

		long long getNumFeaturesInMemory(){
			return numFeaturesInMemory;
		}

		long long getNumRedundantFeatures(){
			return numRedundantFeatures;
		}

		long long getNumSeenObjects(){
			return numSeenObjects;
		}

		double getSmallestDistance(){
			return smallestDistance;
		}

		void setFeatureRedundancyTreshold(double aFeatureRedundancyTreshold){
			featureRedundancyTreshold = aFeatureRedundancyTreshold;
		}

		double getFeatureRedundancyTreshold(){
			return featureRedundancyTreshold;
		}

		void setMaxNumFeaturesInMemory(float MaxNumFeaturesInMemory){
			maxNumFeaturesInMemory = MaxNumFeaturesInMemory;
		}

		float getMaxNumFeaturesInMemory(){
			return maxNumFeaturesInMemory;
		}

		int getNumForgottenFeaturesByMemoryDecayFactor(){
			return nForgottenFeaturesByMemoryDecayFactor;
		}

		// Memory Management
	
		/**
		 * Applies the memory decay factor to all features in memory.
		 * Call this function before adding new features or all features will be affected 
		 * by the decay factor including the new added ones.
		 * Maybe it makes no sense to apply the memory decay factor to fresh new seen features!
		 * The memory decay factor is applied to the local and global weights: 
		 *                  the category and clusters weights of a feature, respectively.
		 *
		 * If after applying the decay memory factor the global weight of a feature drops bellow
		 * a given treshold, that feature will be forgotten.
		 *
		 * Open issues:
		 * 		- The best value for the Memory Decay Threshold has to be determined more precisely and accurately!
		 *		- This should be the main method used to forget features and to control the number of features in memory.
		 *		- This does not, of course, include redundancy!
		 */
		void applyMemoryDecay();

		/**
		 * Gets a feature to forget from memory based on the smallest global weight.
		 * All features in memory are considered except the ones not yet assign to a cluster.
		 * 
		 * returns the id of the feature in memory with the smallest global weight.
		 */
		//int getFeatureToForgetByDecayFactor();
	
	
		/**
		 * Indicates if the application should forget features.
		 */
		void setForgetFeatures(bool forgetFeatures) {this->forgetFeatures = forgetFeatures;};

		/**
		 * Gets information regarding the forgeting mechanism state (on|off) of the application.
		 */
		bool getForgetFeatures() {return forgetFeatures;};
	
		//-->
	
		/**
		 * Get a pointer to Kmeans for update purposes during weight redistribution.
		 */
		void setKmeans(KMeans *aKmeans){
			kmeans = aKmeans;
		}

		/**
		 * Sets debug mode on|off. Debug mode on will print extra information to the console.
		 */
		void setDebugMode(bool state){
			DEBUG = state;
		}

		/* Debug */
		double getMaxSearchRadius() { return maxSearchRadius;}

		double getMinSearchRadius() { return minSearchRadius;}

		double getMaxWg() { return maxWg;}

		double getMinWg() { return minWg;}

		double getMaxWl() { return maxWl;}

		double getMinWl() { return minWl;}

		double getMinDistance() { return minDistance;}

		double getMaxDistance() { return maxDistance;}

		/* Debug */

		/**
		 * Checks redundancy for every feature id specified in the featsIdx vector.
		 * These features are assumed to be new seen features.
		 * This function is currently using snapshots to check redundancy and to redistribute
		 * weights.
		 *
		 * @param[in] featsIdx the id of the features to check for redundancy.
		 */
		void checkRedundancy(vector<int>& featsIdx);

		/**
		 * Sets the search radius used to get the features to redistribute global weights.
		 *
		 * @param[in] GWeights_R_search_radius the search radius value to use.
		 */
		void setGWRedistSearchRadius(double GWeights_R_search_radius)
		{ 
			fixedSearchRadius = GWeights_R_search_radius;
		}

		/**
		 * Gets the search radius used to get the features to redistribute global weights.
		 */
		double getGWRedistSearchRadius()
		{ 
			return fixedSearchRadius;
		}
    protected:
		/**
		 * Checks if a feature is redundant.
		 * Don't use this function. I think, I'm almost sure, it is not checking redundancy 
		 * by itself and mutual redundancy properly.
		 */
		void forgetRedundantFeatures(const vector<int> featuresIds);

		/* Debug */
		double maxSearchRadius;

		double minSearchRadius;

		double maxWg;

		double minWg;

		double maxWl;

		double minWl;

		double minDistance;

		double maxDistance;
		/* Debug */

		double fixedSearchRadius;

		/** 
		 * Redistribute local and global weights. The specified feature is not removed from memory.
		 * This function is called during the feature redundancy check.
		 *
		 * Open issues:
		 *		due to the current architecture, local weight redistribution is not made, because it is not possible
		 *		to apply it to all the features in a category WHEN THE CATEGORY OF THE FEATURE IS UNKNOWNED but only
		 *		on that particular situation. In practice this means that no local weight redistribution is made in
		 *		the case of feature redundancy!
		 *		TODO: change the architecture to: 
		 *				1 - assign new features to a cluster.
		 *				2 - assign new features to a category.
		 *				3 - now we can check those features for redundancy.
		 */
		//void redistributeWeights(int featureId);

		/** 
		 * Removes a feature from memory. 
		 * Updates the cluster sums, unassigns the feature from the respective assigned
		 * cluster and category. Removes the feature from memory, recomputes the cluster
		 * centroid and updates the number of features forgotten and in memory.
		 *
		 * @param[in] featureId the feature id to remove.
		 * @param[in] featureMetadata the features' associated metadata.
		 */
		void remFeatureFromMemory(int featureId, FeatureMetadata &featureMetadata);

		/** 
		 * Redistributes local weights.
		 * The local weight redistribution follows the same formula used for the 
		 * global weight redistribution.
		 *
		 * This function uses a snapshot and should only be used to redistribute local
		 * weights for a set of features belonging to the same category. As is the case
		 * for new seen features of an object view.
		 *
		 * @param[in] toForget the features ids to use for weight redistribution.
		 * @param[in] featureMetadata the category id the features belong to.
		 */
		void redistributeLocalWeights(unordered_set<int> &toForget, int categoryId);

		/** 
		 * Redistribute local weights i.e., to the features belonging to the same category
		 * as the one the feature to forget belongs to.
		 * The local weight redistribution follows the same formula used for the 
		 * global weight redistribution.
		 *
		 * @param[in] featureId the feature id to use for weight redistribution.
		 * @param[in] featureMetadata the features' associated metadata.
		 *
		 */
		void redistributeLocalWeights(int featureId, FeatureMetadata &featureMetadata);

		/** 
		 * Unassigns a feature from a cluster.
		 *
		 * @param[in] clusterId the cluster id.
		 * @param[in] featIdx the feature id.
		 *
		 */
		void removePointFromCluster2(int clusterId, int featIdx);

		/**
		 * Gets a radius to search for neighbors to redistribute global weights.
		 *
		 * If the number of clusters = 0 -> throw exception
		 * If the number of clusters = 1 -> return distance(cluster, feature)		 
		 * If the number of clusters > 1 -> return mean(distance(clusterA, feature) + distance(clusterB, feature)),
		 *				                           clusterA and clusterB are the 2 closest clusters to feature.
		 *
		 * Open issues:
		 *		Idea: get a mean between a max and a min search radius from a set of results and use it as a default, instead of computing this from cluster distances.
		 */
		double getSearchRadius(int featureId, FeatureMetadata &featureMetadata, int k);

		/** 
		 * Unassigns a feature from a category.
		 *
		 * @param[in] categoryId the category id.
		 * @param[in] featIdx the feature id.
		 *
		 */
		void removePointFromCategory2(int categoryId, int featIdx);

		/**
		 * Redistribute global weights to the neighbors of a feature that is going to be forgotten.
		 * The neighbors are found within a given radius.
		 * The weight redistribution follows the formula given in the application specification.
		 *
		 * If no neighbors are found for weight redistribution, the application will continue with
		 * a message to the output.
		 *
		 * Redistribution of global weights is not expected to occur in a situation where there are no clusters yet. If this is to happen the application will throw an exception.
		 *
		 * TODO:
		 *	Check the number of clusters in the begining of the function and 
		 *	throw an exception if it is 0.
		 */
        void redistributeGlobalWeights(int featureId, 
				      FeatureMetadata &featureMetadata, 
				      double searchRadius);

		/**
		 * Redistribute global weights to the neighbors of a feature that is going to be forgotten. This function uses a snapshot.
		 * The neighbors are found within a given radius.
		 * The weight redistribution follows the formula given in the application specification.
		 *
		 * If no neighbors are found for weight redistribution, the application will continue with
		 * a message to the output.
		 *
		 * Redistribution of global weights is not expected to occur in a situation where there are no clusters yet. If this is to happen the application will throw an exception.
		 *
		 * TODO:
		 *	Check the number of clusters in the begining of the function and 
		 *	throw an exception if it is 0.
		 *
		 * @param[in] toForget features ids of features to forget and use to redistribute weights.
		 * @param[in] kdtree the already constructed index to use, the snapshot.
		 * @param[in] mapping features ids mappings with the snapshot.
		 * @param[in] searchRadius the search radius to use.
		 */
		void redistributeGlobalWeights(	unordered_set<int> &toForget, 
					KdTreeFLANN<Histogram<153> > &kdtree, 
					vector<int> &mapping,
					double searchRadius);

		/**
		 * Applies the memory decay factor to a given weight. 
		 * The formula follows: given weight *= memoryDecayFactor
		 *
		 */	    
		float applyMemoryDecayFactor(float weight){
			weight *= memoryDecayFactor;
			return weight;
		}

		/**
		 * Forgets a given feature i.e.,
		 *    1 - redistribute global weights
		 *    2 - redistribute local weights
		 *    3 - update the cluster centroid to wich the feature is assign
		 *    4 - remove the feature from memory
		 *    5 - reorganize the clusters if needed
		 *
		 */
		void forgetFeature(int featureId/*, FeatureMetadata &featureMetadata*/);

		/**
		 * \brief Auxiliar data structure to hold the distances between a specific
		 * object view and the categories. The distance is the euclidean distance
		 * between the respective histograms.
		 *
		 */
	    struct CategoryViewDistance{
			int categoryId;
			double distance;

			CategoryViewDistance(int categoryId, double distance)
				: categoryId(categoryId), distance(distance){}

			bool operator < (const CategoryViewDistance& o) const
			{
				return (distance < o.distance);
			}
	    };

		bool DEBUG;

		KMeans *kmeans;

		double smallestDistance;

		Searcher searcher;

		double featureRedundancyTreshold;

		float memoryDecayFactor;

		float memoryDecayTreshold;

		int nForgottenFeaturesByMemoryDecayFactor;

		bool forgetFeatures;

		long long numFeaturesInMemory;

		long long numSeenFeatures;

		long long numSeenObjects;

		long long numRedundantFeatures;

		long long numForgottenFeatures;

		long long maxNumFeaturesInMemory;

		unordered_map<int, FeatureMetadata> featuresMetadata;
			
		/**
		 * The map of the clusters.
		 */
		unordered_map<int, Cluster> clusters;

		vector<int> newSeenFeaturesIds;

		/**
		 * A sequential generator of integer identifiers for the clusters map structure keys.
		 */
        SeqGenerator clusterIdGen;

        unordered_map<int, Category> categories;

		/**
		 * The map of the categories.
		 */
        unordered_map<std::string, int> categoriesIdMap;
	
        /**
		 * A sequential generator of integer identifiers for the categories map structure keys.
		 */      
        SeqGenerator categoryIdGen;

		/**
		 * A sequential generator of integer identifiers for the features map structure keys.
		 */
		SeqGenerator featureIdGen;
    
    public:

        int getNClusters() {
            return clusters.size();
        }

        unordered_map<int, Cluster> &getClusters() {
            return clusters;
        }

        unordered_map<int, Category> &getCategories() {
            return categories;
        }
    
        /**
         * Get a unique integer identifier for the cluster (map key).
         */
        int generateClusterId() {
            return clusterIdGen.getIdentifier();
        }

        /**
         * Get a unique integer identifier for the category (map key).
         */
        int generateCategoryId() {
            return categoryIdGen.getIdentifier();
        }

        /**
         * Get a unique integer identifier for the feature (map key).
         */
        int generateFeatureId() {
            return featureIdGen.getIdentifier();
        }

        int size() const {
            return featuresMetadata.size();
        }
    
        unordered_map<int, FeatureMetadata> &getFeatures() {
            return featuresMetadata;
        }
};

/**
 * \brief Operator used with std:sort to sort a vector<pair<int, float> >
 * by the value of the pair (the second argument of the pair, the float) in descending order.
 */
struct descending
{
    bool operator()(const std::pair<int, WssBag> &left, const std::pair<int, WssBag> &right) 
    {
        return right.second < left.second;
    }
};
    
#endif  // __MEMORY_H__
