#ifndef __MEASURE_H__
#define __MEASURE_H__

/**
 * \brief A class used to measure the performance of the application assigning object views to categories. 
 * Use to hold the results for a specific category. 
 *
 *
 */
class Measure {
    
    public:
        /**
         * The empty constructor is required when used as the value in a map.
         */   
        Measure();
    
    protected:
        /**
         * True positives
         */
        int tp;
    
        /**
         * False positives
         */
        int fp;
    
        /**
         * False negatives
         */
        int fn;

    public:
	/**
	 * Gets the amount of true positives.
	 */
        int &getTP() {
            return tp;
        }

	/**
	 * Gets the amount of false positives.
	 */
        int &getFP() {
            return fp;
        }

	/**
	 * Gets the amount of false negatives.
	 */
        int &getFN() {
            return fn;
        }

	/**
	 * Increments the amount of true positives.
	 */
        void addTP() {
            tp++;
        }

	/**
	 * Increments the amount of false positives.
	 */
        void addFP() {
            fp++;
        }

	/**
	 * Increments the amount of false negatives.
	 */
        void addFN() {
            fn++;
        }

	/**
	 * Gets the precision.
	 *
	 * returns tp / (tp + fp) or 0 if (tp + fp) = 0.
	 */
        double precision() {
            int den = tp + fp;
            return (den == 0) ? 0 : tp / (double) den;
        }

	/**
	 * Gets the recall.
	 *
	 * returns tp / (tp + fn) or 0 if (tp + fn) = 0.
	 */    
        double recall() {
            int den = tp + fn;
            return (den == 0) ? 0 : tp / (double) den;
        }    
};

#endif  // __MEASURE_H__
