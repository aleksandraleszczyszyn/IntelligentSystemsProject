#ifndef __FEATUREMETADATA_H__
#define __FEATUREMETADATA_H__

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/impl/point_types.hpp>
#include <stdexcept>

/**
 * \brief Holds metadata about a feature as well as the feature itself.
 *
 *
 */
class FeatureMetadata {
    public:
		/**
         * The empty constructor is required when used as the value in a map.
		 * But calling it is considered an error and is treated with an exception!
         */
        FeatureMetadata() { 
	    throw(std::runtime_error("UNEXPECTED CALL TO FeatureMetadata DEFAULT CONSTRUCTOR"));
        }

		/**
		 * Copy constructor.
		 * TODO; is this realy needed?
		 *
		 */
        FeatureMetadata (const FeatureMetadata &obj) {
			lWeight = obj.lWeight;
			gWeight = obj.gWeight;
			count = obj.count;
			cluster = obj.cluster;
			dClusterCentroid = obj.dClusterCentroid;
			category = obj.category;

    	    for(int i=0; i < 153; ++i) {
				feature.histogram[i] = obj.feature.histogram[i];
    	    }
        }

        /**
		 * Creates a new "feature".
		 * Don't use: count and dClusterCentroid!
		 * 
         * @param[in] feature the feature
         * @param[in] lWeight the local weight of the feature (associated to the category 
		 *			  the feature was assigned)
         * @param[in] gWeight the global weight of the feature (associated to the cluster 
	     *            the feature was assigned)
         * @param[in] count the number of similar features the feature represents
         * @param[in] cluster the cluster identifier (map key) to the cluster the feature was assigned to
         * @param[in] dClusterCentroid the distace of the feature to the its cluster centroid
         * @param[in] category the category identifier (map key) to the category the feature 
	                  was assigned to
         */
        FeatureMetadata(pcl::Histogram<153> &feature, 
			float lWeight = 1, float gWeight = 1,
            int count = 1, int cluster = -1, float dClusterCentroid = 0, int category = -1);
    
    protected:

		/**
		* The feature
		*/
		pcl::Histogram<153> feature;
		
		/**
		* The local weight of the feature (features' category weight)
		*/
		float lWeight;
		
		/**
		* The global weight of the feature (in its cluster)
		*/
		float gWeight;
		
    	/**
		* The number of similar features to this feature-
		* Don't use this!
		*
		* This can be used to keep a count of the number of features seen that
		* were similar to this feature and therefore discarded to avoid redundancy
		* This measure can serve as an indicator of how representative a feature is
		* when we have to decide which features to forget
		*/
		int count;
		
		/**
		* The cluster identifier of the cluster the feature has been assigned to
		*/
		int cluster;
		
    	/**
		* The distance from the feature to its cluster centroid.
		* Don't use this!
		*/
		float dClusterCentroid;
		
		/**
		* The category identifier of the category the feature has been assigned to
		*/
		int category;

    public:
	
        /* Getters and Setters */

        float getLocalWeight() const {
            return lWeight;
        }

        void setLocalWeight(float val) {
            lWeight = val;
        }

        float getGlobalWeight() const {
            return gWeight;
        }

        void setGlobalWeight(float val) {
            gWeight = val;
        }

        int getCluster() const {
            return cluster;
        }

        bool assignToCluster(int nCluster) {
            bool changedCluster = cluster != nCluster;
            cluster = nCluster;
            return changedCluster;
        }

        /**
         * Unassigns the feature from the cluster it was assigned to.
         * The value -1 means that the feature is currently not assigned to any cluster.
         */
        void unAssignFromCluster() {
            cluster = -1;
        }

		/** 
		 * Don't use this function!
		 */
        const float getDistanceToCentroid() const {
            return dClusterCentroid;
        }

		/** 
		 * Don't use this function!
		 */
        const int getCount() const {
            return count;
        }
 
		/** 
		 * Returns the feature.
		 */ 
        pcl::Histogram<153> &getFeature() {
            return feature;
        }

		/** 
		 * Don't use this function!
		 */
        void setDistanceToCentroid(float val) {
            dClusterCentroid = val;
        }

        int getCategory() const {
            return category;
        }

        void assignToCategory(int nCategory) {
            category = nCategory;
        }

        /**
         * Unassigns the feature from the category it was assigned to.
         * The value -1 means that the feature is currently not assigned to any category.
         */
        void unAssignFromCategory() {
            category = -1;
        }
    
        void setFeature(pcl::Histogram<153> &feat) {
            feature = feat;
        }

		/** 
		 * Don't use this function!
		 */
        void addCount() {
            ++count;
        }
};

#endif  // __FEATUREMETADATA_H__
