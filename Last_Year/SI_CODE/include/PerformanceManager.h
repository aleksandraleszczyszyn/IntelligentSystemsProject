#ifndef __PERFORMANCEMANAGER_H__
#define __PERFORMANCEMANAGER_H__

#include <vector>
#include "TestResult.h"

/**
 * \brief A class to handle performance results.
 *
 *
 */
class PerformanceManager {
    
    public:
        /**
         * The empty constructor is required when used as the value in a map.
         */
        PerformanceManager() {}

        ~PerformanceManager();

        /**
         * Prints tests performance information.
         */
        void printPerfResults();

        /**
         * Adds a new test result.
         */
        void addTestResult(TestResult *testResult);

        /**
         * Adds a new test result.
         */
        void addTestResult(TestResult &testResult);
    
    protected:
		/**
		 * Auxiliar structure to hold average results.
		 *
		 */
        struct AverageResults{
    	    double precision;
			double recall;
			int nTests;

			AverageResults(double precision, double recall) 
			: precision(precision), recall(recall), nTests(1) {};

			AverageResults(){};
        };

        /**
         * Holds the perfomance results (precision and recall) for the categories used in 
		 * all the test runs.
         */
        std::vector<TestResult* > testResults;

        /**
         * Holds the perfomance results (precision and recall) for the categories used in 
		 * all the test runs.
         */
        std::vector<TestResult> perf;
 
    public:
        const std::vector<TestResult> &getResults() {
            return perf;
        }

};

#endif  // __PERFORMANCEMANAGER_H__
