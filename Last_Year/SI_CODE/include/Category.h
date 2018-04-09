#ifndef __CATEGORY_H__
#define __CATEGORY_H__

#include <set>
#include <vector>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include "Measure.h"

#include <tr1/unordered_set>

using std::tr1::unordered_set;

/**
 * \brief Represents an object category, e.g., apple, bowl, bell-pepper, etc.
 *
 * Open issues:
 *    - a category should hold/handle its' own histogram. In this version
 *      the category' histogram is computed every time a new view-category
 *	    prediction is made. A better approach could hold and maintain the
 *		histogram instead of computing it every time.
 */
class Category {
    
    public:
		/**
		 * The empty constructor is required when used as the value in a map.
		 * However! a call to this constructor could be due to an error so
		 * calling this constructor is considered a severe error and it is treated
		 * with an exception! No "not well formed object" are allowed to be injected into
		 * the applications' stream!
		 */
		Category();

        /**
		 * Creates a new category.
		 *
         * @param[in] identifier the unique identifier of the category
         * @param[in] name the name of the category, e.g., apple, tomato, etc.
         */
		Category(int identifier, std::string &name);

        /**
         * Adds a feature to the category.
		 * 
    	 * @param[in] idx the id of the feature to add (features' id in the main memory structure).
         */
		void addPoint(int idx);

        /**
         * Removes a feature from the category. 
		 * Note: The feature is not removed from the main memory!
         * 
         * @param[in] idx the id of the feature to remove (features' id in the main memory structure).
         */
		void removePoint(int idx);

        /**
		 * Gets the features assigned to the category, i.e., 
		 * the indeces of the features in main memory.
		 *
		 * This is a read only method and it is probably not even used.
		 * TODO: remove this method?
		 *
         * @param[out] pointsIdx a vector holding the indeces (map keys) of the features 
		 * assigned to the category.
         */
		void getPoints(std::vector<int> &pointsIdx) const;

        /**
		 * A string representation, of the category, of the form: 
		 * 		category_name Id: category_id P: unordered_list_of_point_ids
		 *
		 *		ex:
		 *			apple Id: 1 P: 10, 12, 5, ...,
		 *
		 * Notes: 
		 *		Used for debug purposes.
         */
        std::string toString() const;

        /* Getters */

        /**
	  	 * Gets the name of the category.
         */
    	const std::string &getName() {
            return name;
    	}

        /**
	  	 * Gets the category identifier.
		 *
		 * Issues: Is this identifier realy needed? 
		 * 		   It is the same as the map key in the main memory!
		 *	 	   TODO: remove this?
         */
    	const int getIdentifier() const {
            return identifier;
    	}

        /**
		 * Gets the feature indeces assigned to the category.
         */
    	unordered_set<int> &getPoints() {
            return points;
    	}	
    
	    /* Category Test Performance Measurement Functions */
	
        /**
		 * Gets the Measure class with information about the test results 
		 * obtained for the category.
         */
        Measure &getPerfMeasure() {
            return perfAssess;
        }

        /**
		 * Increments true positives.
         */
        void addTP() {
            perfAssess.addTP();
        }

        /**
		 * Increments false positives.
         */
        void addFP() {
            perfAssess.addFP();
        }
        /**
		 * Increments false negatives.
         */
        void addFN() {
            perfAssess.addFN();
        }

        /**
		 * Gets the test precision value for the category.
         */
        double precision() {
            return perfAssess.precision();
        }

        /**
		 * Gets the test recall value for the category.
         */
        double recall() {
            return perfAssess.recall();
        }   

    protected:
        /**
         * The category unique identifier.
         */
    	int identifier;

        /**
         * The category name.
         */
    	std::string name;

        /**
         * The features' ids assigned to the category (the map keys of the features in main memory).
         */
    	unordered_set<int> points;

        /**
         * Holds the recognition performance measures for the category.
         */
        Measure perfAssess;

    private:
		//
};

#endif  // __CATEGORY_H__
