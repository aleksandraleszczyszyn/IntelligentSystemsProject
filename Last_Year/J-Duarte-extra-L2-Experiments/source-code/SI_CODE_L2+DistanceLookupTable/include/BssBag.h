#ifndef __BSS_BAG_H__
#define __BSS_BAG_H__

#include <set>
#include <vector>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include "Measure.h"

/**
 * \brief Utility class to hold several mertadata about bss values
 *
 */
class BssBag {
    
    public:
		
	/**
	 * The empty constructor is required when used as the value in a map.
	 */
	BssBag();
	
	int getId() {
            return id;
	}
	
	void setId(const int &value) {
            id = value;
	}
	
	float getBss() {
            return bss;
	}

	void setBss(float &value) {
            bss = value;
	}
	
	int getCount() {
            return count;
	}
	
	void setCount(const int &value) {
            count = value;
	}
	
    protected:
 
		/**
         * A unique identifier.
         */
    	int id;

        /**
         * The bss.
         */
    	float bss;

        /**
         * A count.
         */
        int count;

};

#endif  // __BSS_BAG_H__
