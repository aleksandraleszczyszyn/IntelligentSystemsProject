#ifndef __CLUSTER_H__
#define __CLUSTER_H__

#include <set>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <stdexcept>

#include <tr1/unordered_set>

using std::tr1::unordered_set;

/**
 * \brief Represents a cluster.
 */
class Cluster {
    
    public:
        /**
         * The empty constructor is required when used as the value in a map.
		 * However calling it is considered an error since an invalid object will be injected into the system!
         */  
        Cluster() { 
	    throw(std::runtime_error("UNEXPECTED CALL TO Cluster DEFAULT CONSTRUCTOR"));
        }

        /**
         * @param[in] identifier the unique identifier of the cluster
         * @param[in] center the centroid of the cluster
         */   
        Cluster(int identifier, pcl::Histogram<153> &center);
    
        /**
         * Sets the centroid of the cluster.
         * @param[in] center the centroid of the cluster.
         */
        void setCentroid(pcl::Histogram<153> &center);
    
        /**
         * Adds a feature to the cluster (the index of the feature in main memory is used).
         * @param[in] idx the index of the feature in the main memory structure dictionary (its key).
         */    
        void addPoint(int &idx);

        /**
         * Removes a feature from the cluster (the index of the feature in main memory is used).
         * @param[in] idx the index of the feature in the main memory structure dictionary (its key).
         */
        void removePoint(int &idx);

        /**
         * Remove all points from the list of popints assigned to the cluster.
         */
        void clearPoints();
    
        /**
         * Return the features assigned to the cluster (the indeces of the features in main memory are returned).
         * @param[out] pointsIdx a vector holding the indeces (map keys) of the features assigned to the cluster.
         */
        void getPoints(std::vector<int> &pointsIdx) const;

		/**
		* Get a random point index from the set of points in the cluster
		*/
		const int getRandomPoint();

		/**
		* Get a random point index from the set of points in the cluster
		*
		* @param[in] pointIdx the value not to duplicate
		*
		* Returns random number but different from the specified value
		*/
		const int getRandomPoint(int &pointIdx);
		
		/**
		* Adds a new weight to the running sum of weights
		*
		* @param[in] weight the new weight to add
		*/
		float &addToSumWeight(const float &weight);

		/**
		* Subtracts a weight from the running sum of weights
		*
		* @param[in] weight the new weight to subtract
		*/
		float &subtractFromSumWeight(const float &weight);

		/**
		* Adds the product (feat * featWeight) of a new feature to the running weighted sum of features
		*
		* @param[in] feat the feature
		* @param[in] featWeight the global weight of the feature
		*/	
		pcl::Histogram<153> &addToSumFeatures(pcl::Histogram<153> &feat, const float &featWeight);
		
		/**
		* Subtracts the product (feat * featWeight) of a new feature from the running weighted sum of features
		*
		* @param[in] feat the feature
		* @param[in] featWeight the global weight of the feature
		*/
		pcl::Histogram<153> &removeFromSumFeatures(pcl::Histogram<153> &feat, const float &featWeight);
		
		/**
		* Recompute the cluster centroid based on the running sums (sumFeatures / sumWeight)
		*/
		void recomputeCentroid();
		
		/**
		* Resets the running totals of the cluster to zeroes; sumWeight and sumFeatures
		*/
		void resetClusterTotals();
		
    protected:

		/**
		* The cluster identifier
		*/
		int identifier;
		
		/**
		* The centroid of the cluster
		*/
		pcl::Histogram<153> centroid;
		
		/**
		* The features assigned to the cluster (the map keys of the features in main memory)
		*/
		//std::set<int> points;

		unordered_set<int> points;

		/**
		* The sum of the global weights is maintained along with the sum of the (weight * feature) calculation
		*
		* This is done so that updating the centroids is less computationally intensive
		*
		* The centroid is calculated by dividing the sumFeatures by the sumWeight (sumFeatures / sumWeight)
		*
		* When a new feature is added to the cluster its weight s added to the sum of weights (sumWeight)
		* The product (weight * feature) is also added to the weighted sum of the features (sumFeatures)
		*
		* To remove the feature from the cluster the process is similar, but instead of adding to the
		* running  summations, we subtract
		*/
		float sumWeight;

		/**
		* The sum of the features mulyiplied by their respective global weights
		*/ 
		pcl::Histogram<153> sumFeatures;
		   
		/**
		* The WSS measure of the cluster
		*/
		float wss;
		
		/**
		* The BSS measure of the cluster
		*
		* p.s. This is not a proper BSS. This is just to help calculations
		*/
		float bss;
		
    public:

        pcl::Histogram<153> &getCentroid() {
            return centroid;
        }

        const int &getIdentifier() const {
            return identifier;
        }

        unordered_set<int> &getPoints() {
            return points;
        }

	/*
        std::set<int> &getPoints() {
            return points;
        }
	*/

        int size() {
            return points.size();
        }

        float &getSumWeight() {
            return sumWeight;
        }

        /**
         * Resets the running total for weights.
         */
        void resetSumWeight() {
            sumWeight = 0;
        }
    
        pcl::Histogram<153> &getSumFeatures() {
            return sumFeatures;
        }

        void resetSumFeatures() {
            for(int i=0; i < 153; ++i) {
                sumFeatures.histogram[i] = 0;
            }
        }

        float &getWss() {
            return wss;
        }

        void setWss(float &val) {
            wss = val;
        }
    
        float &getBss() {
            return bss;
        }

        void setBss(float &val) {
            bss = val;
        }
};

#endif  // __CLUSTER_H__
