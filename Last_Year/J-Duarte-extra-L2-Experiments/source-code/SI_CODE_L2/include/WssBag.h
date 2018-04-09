#ifndef __WSS_BAG_H__
#define __WSS_BAG_H__

#include <math.h>
//#include <set>
//#include <vector>
//#include <iostream>
//#include <sstream>
//#include <stdexcept>

//#include <pcl/point_types.h>
//#include <pcl/pcl_base.h>

//#include "Measure.h"

/**
 * \brief Utility class to hold numeric values
 *
 */
class WssBag {
    
    public:
		
	/**
	 * The empty constructor is required when used as the value in a map.
	 */
	WssBag();

	bool operator<(const WssBag& bag) const{
		return getNormalizedWss() < bag.getNormalizedWss();	
	}
	
	int getId() {
            return id;
	}
	
	void setId(const int &value) {
            id = value;
	}

	float getWss() const {
            return wss;
	}
	
	float getNormalizedWss() const {
            return count != 0 ? wss / count : 0;
	}
	
	void setWss(float &value) {
            wss = value;
	}
	
	int getCount() {
            return count;
	}
	
	void setCount(const int &value) {
            count = value;
	}
	
	float getClusterRadius() {
            return clusterRadius;
	}

	void setClusterRadius(const float &value) {
            clusterRadius = value;
	}
	
	float getClusterDensity() {
		//return clusterRadius != 0 ? count / clusterRadius : 0;
		//return count / (M_PI * (clusterRadius * clusterRadius));
		return clusterRadius != 0 ? count / (clusterRadius * clusterRadius) : 0;
	}
	
	void setMinDistance(const float &value) {
            minDistance = value;
	}
	
	float getMinDistance() {
		return minDistance;
	}
	
	void setMaxDistance(const float &value) {
            maxDistance = value;
	}
	
	float getMaxDistance() {
		return maxDistance;
	}
	
/*	void setAsBssCandidate(const bool &value) {
            bssCandidate = value;
	}
	
	bool isBssCandidate() {
		return bssCandidate;
	}*/
	
	void setOperatedOn() {
            operatedOn = true;
	}
	
	bool asBeenOperatedOn() {
		return operatedOn;
	}
	
    protected:
 
		/**
         * A unique identifier.
         */
    	int id;

        /**
         * The wss.
         */
    	float wss;
	
        /**
         * The minimum distance to the cluster centroid.
         */
    	float minDistance;

        /**
         * The maximum distance to the cluster centroid.
         */
    	float maxDistance;	
	
        /**
         * A count.
         */
        int count;

		float clusterRadius;
	
		//bool bssCandidate;
	
		bool operatedOn;
};

#endif  // __WSS_BAG_H__
