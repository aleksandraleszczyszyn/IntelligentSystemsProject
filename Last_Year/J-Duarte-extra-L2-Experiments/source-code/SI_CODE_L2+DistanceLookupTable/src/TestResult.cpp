#include "TestResult.h"
#include <iomanip>

using namespace std;

TestResult::TestResult()
    : nSamples(0), nCorrectClass(0), nClusters(0) 
{}

void TestResult::setPerfAssess(unordered_map<int, Category> &categories) {
    for(unordered_map<int, Category>::iterator cat=categories.begin(); cat != categories.end(); ++cat) {
        perfAssess.push_back(make_pair(cat->second.getName(), cat->second.getPerfMeasure()));
    }
}

void TestResult::printTestResults() {
    // print the test results
    cout << "Test performance" << endl;
    cout << "\t" << nClusters << " cluster(s) used in the test" << endl;
    cout << "\t" << nSamples << " samples used: " << nCorrectClass 
	         << " correct classifications, accuracy: " 
		 << setprecision(2) << fixed << accuracy() * 100 << "%" << endl;

    cout << "Category Meassures" << endl;
    for(vector<pair<string, Measure> >::iterator mes=perfAssess.begin(); mes != perfAssess.end(); ++mes) {
        cout << "\t" << "Category " << mes->first 
             << " ,precision " << setprecision(2) << fixed << mes->second.precision() * 100 << "%"
             << " ,recall " << setprecision(2) << fixed << mes->second.recall() * 100 << "%"
             << endl;
    }

    // print memory managment information
    cout << "Memory Managment" << endl;
    cout << "\t" << "Nº forgotten features: " << getNumForgottenFeatures() << endl;
    cout << "\t" << "Nº redundant features: " << getNumRedundantFeatures() << endl;
    cout << "\t" << "Nº forgotten features by Memory Decay: " << getNumForgottenFeaturesByMemDecay() << endl;
    cout << "\t" << "Nº seen objects: " << getNumSeenObjects() << endl;
    cout << "\t" << "Nº seen features: " << getNumSeenFeatures() << endl;
    cout << "\t" << "Nº features in memory: " << getNumFeaturesInMemory() << endl;
    cout << "\t" << "Feature smallest distance: " << setprecision(6) << fixed 
	 << getSmallestDistance() << endl;
}
