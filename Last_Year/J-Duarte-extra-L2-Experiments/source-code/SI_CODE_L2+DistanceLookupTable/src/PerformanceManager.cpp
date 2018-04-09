#include "PerformanceManager.h"
#include <iomanip>

using namespace std;

PerformanceManager::~PerformanceManager() {
    for(int i = 0; i < testResults.size(); i++) {
        delete testResults.at(i);
    }
}

void PerformanceManager::printPerfResults() {
    double meanAccuracy = 0.0;
    map<string, AverageResults> categoryAverageResults;
    std::vector<std::pair<std::string, Measure> >* categoryPerformanceMeassures;

    TestResult* testResult;
    AverageResults* averageResults;

    cout << endl << "RESULTS" << endl;

    // get results
    for(int i = 0; i < testResults.size(); i++){
	testResult = testResults.at(i);

	cout << "Test " << i + 1 << endl;

	testResult->printTestResults();
	meanAccuracy += testResult->accuracy();

	categoryPerformanceMeassures = &testResult->getCategoryPerformanceMeassures();
        
        for(vector<pair<string, Measure> >::iterator 
		mes = categoryPerformanceMeassures->begin(); 
		mes != categoryPerformanceMeassures->end(); ++mes) {

    		std::map<string, AverageResults>::iterator it = 
							categoryAverageResults.find(mes->first);

    		if(it == categoryAverageResults.end()){
		    categoryAverageResults[mes->first] = 
				AverageResults(mes->second.precision(), mes->second.recall());
               }else{
		    averageResults = &categoryAverageResults[mes->first];

		    averageResults->precision += mes->second.precision();
		    averageResults->recall += mes->second.recall();
		    averageResults->nTests += 1;
		}
        }
	cout << endl;
    }

    meanAccuracy /= testResults.size();

    // average results
    cout << "Average Results" << endl;
    cout << "\t" << "Accuracy: " << setprecision(2) << fixed << meanAccuracy * 100 << "%" << endl;

    for(map<string, AverageResults>::iterator it = categoryAverageResults.begin(); 
					      it != categoryAverageResults.end(); ++it) {
	cout << "\t" << "Category: " << it->first 
		     << " Precision: " 
		     << setprecision(2) << fixed << it->second.precision / it->second.nTests * 100 << "%"
		     << " Recall: " 
		     << setprecision(2) << fixed << it->second.recall / it->second.nTests * 100  << "%"
		     << endl;
    }

    cout << endl;
}

void PerformanceManager::addTestResult(TestResult &testResult) {
    perf.push_back(testResult);
}

void PerformanceManager::addTestResult(TestResult* testResult) {
    testResults.push_back(testResult);
}
