#ifndef __TESTRESULT_H__
#define __TESTRESULT_H__

#include <vector>
#include <map>
#include <string>
#include "Measure.h"
#include "Category.h"

#include <tr1/unordered_map>

using std::tr1::unordered_map;

/**
 * \brief Holds the performance test results for individual test runs.
 *
 *
 */
class TestResult {
    
    public:
	    /**
	     * The empty constructor is required when used as the value in a map.
	     */    
	    TestResult();

	    /**
	     * Sets the performance measures obtained for the categories.
	     */
	    void setPerfAssess(unordered_map<int, Category> &categories);

	    /**
	     * Prints test perfomance information.
	     */
	    void printTestResults();
    
    protected:
	    /**
	     * Holds the perfomance results (precision and recall) for the categories used in 
	     * the test run.
	     */
	    std::vector<std::pair<std::string, Measure> > perfAssess;

	    /**
	     * The number of samples used in the test run.
	     */
	    int nSamples;

	    /**
	     * The number of correct classifications occored during the test run.
	     */
	    int nCorrectClass;

	    /**
	     * The number of clusters used to perfom the test.
	     */
	    int nClusters;
	    
	    /**
	     * The number of forgotten features.
	     */
	    long long numForgottenFeatures;

	    /**
	     * The number of redundant features.
	     */
	    long long numRedundantFeatures;

	    /**
	     * The number of forgotten features by reaching the memory decay factor.
	     */
	    long long numForgottenFeaturesByMemoryDecayFactor;

	    /**
	     * The number of seen objects.
	     */
	    long long numSeenObjects;

	    /**
	     * The number of seen features.
	     */
	    long long numSeenFeatures;

	    /**
	     * The final number of features in memory.
	     */
	    long long numFeaturesInMemory;

	    /**
	     * The smmalest distance between 2 features during the test.
	     */
	    double smallestDistance;

    public:

	    void setNumForgottenFeatures(long long NumForgottenFeatures){
			numForgottenFeatures = NumForgottenFeatures;
	    }

	    long long getNumForgottenFeatures(){
			return numForgottenFeatures;
	    }

	    void setNumForgottenFeaturesByMemDecay(long long NumForgottenFeaturesByMemDecay){
			numForgottenFeaturesByMemoryDecayFactor = NumForgottenFeaturesByMemDecay;
	    }

	    long long getNumForgottenFeaturesByMemDecay(){
			return numForgottenFeaturesByMemoryDecayFactor;
	    }

	    void setNumRedundantFeatures(long long NumRedundantFeatures){
			numRedundantFeatures = NumRedundantFeatures;
	    }

	    long long getNumRedundantFeatures(){
			return numRedundantFeatures;
	    }

	    void setNumSeenObjects(long long NumSeenObjects){
			numSeenObjects = NumSeenObjects;
	    }

	    long long getNumSeenObjects(){
			return numSeenObjects;
	    }

	    void setNumSeenFeatures(long long NumSeenFeatures){
			numSeenFeatures = NumSeenFeatures;
	    }

	    long long getNumSeenFeatures(){
			return numSeenFeatures;
	    }

	    void setNumFeaturesInMemory(long long NumFeaturesInMemory){
			numFeaturesInMemory = NumFeaturesInMemory;
	    }

	    long long getNumFeaturesInMemory(){
			return numFeaturesInMemory;
	    }

	    void setSmallestDistance(double SmallestDistance){
			smallestDistance = SmallestDistance;
	    }

	    double getSmallestDistance(){
			return smallestDistance;
	    }

	    std::vector<std::pair<std::string, Measure> >& getCategoryPerformanceMeassures(){
			return perfAssess;
	    }    

	    /**
	     * Returns the accuracy in the range 0-1
	     */
	    double accuracy() {
			return nCorrectClass / (double)nSamples;
	    }

	    void addToSample() {
			nSamples++;
	    }

	    void addToCorrectClassifications() {
			nCorrectClass++;
	    }

	    /**
	     * Gets the final number of clusters used during the test.
	     */
	    int &getNumClusters() {
			return nClusters;
	    }

	    void setNumClusters(int nClusts) {
			nClusters = nClusts;
	    }
};

#endif  // __TESTRESULT_H__
