#include <iostream>
#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <stdlib.h> 	//for atoi
#include <algorithm>
#include <functional>
#include <iomanip>
#include <fstream>

// pcl includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/pcl_base.h>

// custom includes
#include <include.h>
#include <Viewer.h>
#include "kmeans.h"
#include "Memory.h"
#include "Cluster.h"
#include "PerformanceManager.h"
#include <FeatureExtractor.h>
#include <ObjectViewRepository.h>
#include <CategoryRecognizer.h>


#include<time.h> 	// for clock
#include<math.h> 	// for fmod
#include<cstdlib> 	// for system
#include <stdio.h> 	// for delay
//#include <thread>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
using boost::thread;

using namespace std;
using namespace pcl;
using namespace pcl::io;

// application paremeters
float voxel_size = 0.03;	// 0.015 or 0.03 meters

// spin images parameters
float search_radius = 0.1;	// 0.1 meters
int window_width = 8;
float support_lenght = 0.1;	// 0.1 meters
float support_angle = 0;	// value in [0, 1], related to the cosine of the angle cos(90º) = 0
float min_pts_neighbours = 0;

double featureRedundancyTreshold;
float memoryDecayFactor;
float memoryDecayTreshold;
long long maxNumFeaturesInMemory;

float redistribute_gWeights_search_radius = 0.05;

bool DEBUG = false;		// debug/verbose mode

const char* object_views_directory = NULL;
int nCategories = 0;
float trainingPercentage = 0.0;
int numberOfTests = 0;
const char* configuration_file = NULL;
bool forget = false;
int nViewsToSeePerCategory = 0;
bool randShuffle = false;
const int numberOfThreads = 2;
const int numIters = 10;

/**
 * Loads the configuration file from disk.
 *
 *
 */
bool load_configuration(const char* configuration_file){
    ifstream fin(configuration_file);
    string line;
    istringstream sin;

    while (getline(fin, line)) {
		sin.str(line.substr(line.find("=") + 1));

		if (line.find("sp_search_radius") != string::npos) {
			sin >> search_radius;
		}
		else if (line.find("voxel_size") != string::npos) {
			sin >> voxel_size;
		}
		else if (line.find("rGW_search_radius") != string::npos) {
			sin >> redistribute_gWeights_search_radius;
		}
		else if (line.find("window_width") != string::npos) {
			sin >> window_width;
		}
		else if (line.find("support_lenght") != string::npos) {
			sin >> support_lenght;
		}
		else if (line.find("support_angle") != string::npos) {
			sin >> support_angle;
		}
		else if (line.find("min_pts_neighbours") != string::npos) {
			sin >> min_pts_neighbours;
		}
		else if (line.find("featureRedundancyTreshold") != string::npos) {
			sin >> featureRedundancyTreshold;
		}
		else if (line.find("memoryDecayFactor") != string::npos) {
			sin >> memoryDecayFactor;
		}
		else if (line.find("memoryDecayTreshold") != string::npos) {
			sin >> memoryDecayTreshold;
		}
		else if (line.find("maxNumFeaturesInMemory") != string::npos) {
			sin >> maxNumFeaturesInMemory;
		}

		sin.clear();
    }
}

/**
 * Prints the application configuration.
 *
 */
void printConfiguration(){
    cout << endl << "Configuration" << endl;
    cout << "\t" << "voxel_size " << voxel_size << endl;
    cout << "\t" << "spin_imgs_search_radius " << search_radius << endl;
    cout << "\t" << "window_width " << window_width << endl;
    cout << "\t" << "support_length " << support_lenght << endl;
    cout << "\t" << "support_angle " << support_angle << endl;
    cout << "\t" << "min_pts_neighbours " << min_pts_neighbours << endl;
    cout << "\t" << "redistribute_GWeights_search_radius " << redistribute_gWeights_search_radius << endl;

    if(forget)
		cout << "\t" << "Forget Features On" << endl;
    else 
		cout << "\t" << "Forget Features Off" << endl;

    cout << "\t" << "Number Of Tests To Perform: " << numberOfTests << endl;
    cout << "\t" << "Number Categories used in the tests: " << nCategories << endl;
    cout << "\t" << "% Views for training: " << trainingPercentage << endl;
    cout << "\t" << "Views Dir: " << object_views_directory << endl;
    cout << "\t" << "Memory Decay Factor: " << memoryDecayFactor << endl;
    cout << "\t" << "Memory Decay Threshold: " << memoryDecayTreshold << endl;
    cout << "\t" << "Feature Redundancy Threshold: " << featureRedundancyTreshold << endl;
    cout << "\t" << "Max Number Features In Memory: " << maxNumFeaturesInMemory << endl;

    double bin_size = search_radius / window_width / sqrt(2.0);
    const float r = search_radius * window_width;

    cout << "\t" << "bin_size " << bin_size << endl;
    cout << "\t" << "cylinder r = " << r << " height = " << 2 * r << endl;
    cout << endl;
}

/**
 * Runs a test. A test includes 2 phases: trainig and learning.
 *
 *
 */


float RunThreads(const int numberOfClusters, Memory* memory, float* Wss, KMeans *kMeans, ObjectViewRepository* objectViewRepository) {

    //Memory memory;
    TestResult *testResult = new TestResult();
    FeatureExtractor featureExtractor = FeatureExtractor();
//    ObjectViewRepository objectViewRepository = ObjectViewRepository(DEBUG, nViewsToSeePerCategory, randShuffle);

    char callResult = objectViewRepository->SetObjectViewRepositorySource(object_views_directory);
    if (callResult == 2) {
        cout << "Error loading the specified views repository." << endl;
        return 0;
    } else if (callResult == 3) {
        cout << "Error reading file/diretory in the views repository." << endl;
        return 0;
    } else if (callResult == 4) {
        return 0;
    }

    if (DEBUG) {
        objectViewRepository->printCategories();
        objectViewRepository->printCategoryViews();
    }

    objectViewRepository->shuffleViewsForTraining(nCategories, trainingPercentage);

    char state = 0; //training



    memory->setMemoryDecayFactor(memoryDecayFactor);
    memory->setFeatureRedundancyTreshold(featureRedundancyTreshold);
    memory->setMaxNumFeaturesInMemory(maxNumFeaturesInMemory);
    memory->setMemoryDecayTreshold(memoryDecayTreshold);
    memory->setDebugMode(DEBUG);
    memory->setForgetFeatures(forget);
    memory->setGWRedistSearchRadius(redistribute_gWeights_search_radius);

    //KMeans kMeans(numIters, numberOfClusters);


    kMeans->init(*memory);
    memory->setKmeans(kMeans);

    ostringstream category;
    vector<double> *viewHistogram;
    int categoryId;
    int predictedCategoryId;
    int featId;
    char ret;

    cout << endl << "Task: Training" << endl << endl;

    // learning and training cicle
    while (1) {
        PointCloud<PointT>::Ptr pc(new PointCloud<PointT>);
        ret = objectViewRepository->NextObjectView(pc);

        if (ret == 3) {
            cout << "The ObjectViewRepository is not initialized yet!" << endl;
            break;
        } else if (ret == 2) {
            if (state == 0) {
                cout << endl;
                cout << "Training done!" << endl;
                cout << endl;

                // learning
                cout << "Starting Testing phase!" << endl;
                objectViewRepository->shuffleViewsForLearning();

                cout << endl;
                cout << "Task: Testing" << endl;
                cout << endl;

                state = 1;
                //TODO create class with all parameters and return it
                // holds the wss computed for each cluster
                vector<pair<int, WssBag> > clustersWss;
                // wss is the total WSS computed over all clusters
                // WSS stands for Within cluster Sum of Squares
                *Wss = memory->wss(clustersWss);
                cout<<"THREAD WSS "<<*Wss<<" ??????????????????????????????????????????????????????????????????????????????????????????????????????????????"<<endl;
                cout << memory->getNClusters() << "Number of clustersssssssssssssssssssssss *******************************************" << endl;
                cout << "kMeanssssssssssss" << kMeans->numOfClusters << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;

                return 0;
            } else {
                cout << endl << "Testing done!" << endl;
                break;
            }
        } else if (ret == 5) {
            cout << "No enough categories available!" << endl;
            break;
        } else if (ret == 0) {
            cout << "Error loading object view!" << endl;
            break;
        }

        // get current view category
        category.str("");
        category.clear();
        category << objectViewRepository->getCategory();

        // training
        if (state == 0) {
            categoryId = memory->addCategory(category.str());
            // learning
        } else {
            categoryId = memory->getCategoryId(category.str());
        }

        if (DEBUG)
            cout << "CATEGORY: " << category.str() << " ID: " << categoryId << endl;

        // keypoints
        PointCloud<PointT>::Ptr keypoints(new PointCloud<PointT>);

        // Get keypoints
        featureExtractor.GetKeypointsUsingAVoxelFilter(pc, voxel_size, keypoints);

        // Spin images
        PointCloud<SpinImage>::Ptr spin_images(new PointCloud<SpinImage>);

        // Compute the keypoints' spin images
        featureExtractor.ComputeSpinImagesAtKeypoints(pc, keypoints,
                                                      search_radius,
                                                      window_width, support_lenght,
                                                      support_angle, min_pts_neighbours, spin_images);

        // Apply memory decay factor. Call before adding new features!
        memory->applyMemoryDecay();

        vector<int> featsIdx;

        for (size_t i = 0; i < spin_images->points.size(); ++i) {
            // add feature to memory if not redundant
            callResult = memory->addFeature(spin_images->points[i], featId);

            // spin image with nan
            if (callResult == 2 /*|| callResult == 3*/) {
                continue;
            }

            // if training, add feature to category
            if (state == 0 /*&& callResult == 1*/) {
                memory->addPointToCategory(categoryId, featId);
            }

            // keep the ids (map keys) of the features added to memory
            featsIdx.push_back(featId);
        }

        // assign the new features to clusters and do kmeans needed operations
        kMeans->assign(featsIdx);

        memory->checkRedundancy(featsIdx);

    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    // holds the wss computed for each cluster
//    vector<pair<int, WssBag> > clustersWss;
//
//    // wss is the total WSS computed over all clusters
//    // WSS stands for Within cluster Sum of Squares
//    Wss = memory.wss(clustersWss);
//    cout<<"THREAD WSS "<<Wss<<" ??????????????????????????????????????????????????????????????????????????????????????????????????????????????"<<endl;

    //dataFromThreads[threadCounter]->wss=avgWss;
    return 0;
}


Memory runTest(TestResult &testResult) {


        Memory memoryThr[numberOfThreads];
        float Wss[numberOfThreads];

        int numberOfClusters=6;

        boost::thread *threads[numberOfThreads];

        KMeans kMeans1(numIters,numberOfClusters+0);
        KMeans kMeans2(numIters,numberOfClusters+1);
        KMeans kMeans3(numIters,numberOfClusters+2);
        KMeans kMeans4(numIters,numberOfClusters+3);

        ObjectViewRepository objectViewRepository1 = ObjectViewRepository(DEBUG, nViewsToSeePerCategory, randShuffle);
        ObjectViewRepository objectViewRepository2 = ObjectViewRepository(DEBUG, nViewsToSeePerCategory, randShuffle);
        ObjectViewRepository objectViewRepository3 = ObjectViewRepository(DEBUG, nViewsToSeePerCategory, randShuffle);
        ObjectViewRepository objectViewRepository4 = ObjectViewRepository(DEBUG, nViewsToSeePerCategory, randShuffle);

        for (int i = 0; i < numberOfThreads; i++) {
            if (i==0)
            {

                threads[i] = new thread(RunThreads, numberOfClusters+i, &memoryThr[i], &Wss[i], &kMeans1, &objectViewRepository1);
            }
            if (i==1)
            {
                threads[i] = new thread(RunThreads, numberOfClusters+i, &memoryThr[i], &Wss[i], &kMeans2, &objectViewRepository2);
            }
            if (i==2)
            {
                threads[i] = new thread(RunThreads, numberOfClusters+i, &memoryThr[i], &Wss[i], &kMeans3, &objectViewRepository3);
            }
            if (i==3)
            {
                threads[i] = new thread(RunThreads, numberOfClusters+i, &memoryThr[i], &Wss[i], &kMeans4, &objectViewRepository4);
            }
        }

        for (int i = 0; i < numberOfThreads; i++) {
            threads[i]->join();
        }


        cout << "All threads finished!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        for (int i=0; i<numberOfThreads; i++)
        {
            cout << "smallestDistance: " << Wss[i] << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            cout << memoryThr[i].getNClusters() << "Number of clustersssssssssssssssssssssss @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;

        }
    cout << "kMeanssssssssssss" << kMeans1.numOfClusters << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
    cout << "kMeanssssssssssss" << kMeans2.numOfClusters << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
        //TODO get best thread

//        float a = threads[0]
//
//        //TODO get CategoryRecognizer,KMeans,ObjectViewRepository,Memory from best thread
        //CategoryRecognizer ------------------------------------- done
                //KMeans ----------------------------------------- done
                        //ObjectViewRepository
                                // Memory ------------------------ done
                                        // Wss ------------------- done
//
        FeatureExtractor featureExtractor = FeatureExtractor();


        //// here chose the best thread
    CategoryRecognizer categoryRecognizer(memoryThr[0], DEBUG);
    ObjectViewRepository objectViewRepository = objectViewRepository1;
    Memory memory = memoryThr[0];
    KMeans kMeans = kMeans1;




        if (DEBUG) {
            objectViewRepository.printCategories();
            objectViewRepository.printCategoryViews();
        }


        char state = 1; //testing
        const int numIters = 10;
        const int numOfClusters = 7;


        ostringstream category;
        vector<double> *viewHistogram;
        int categoryId;
        int predictedCategoryId;
        int featId;
        char ret;

        cout << endl << "Task: Testing" << endl << endl;

        // learning and training cicle
        while (1) {
            PointCloud<PointT>::Ptr pc(new PointCloud<PointT>);
            ret = objectViewRepository.NextObjectView(pc);

            if (ret == 3) {
                cout << "The ObjectViewRepository is not initialized yet!" << endl;
                break;
            } else if (ret == 2) {

                    cout << endl << "Testing done!" << endl;
                    break;

            } else if (ret == 5) {
                cout << "No enough categories available!" << endl;
                break;
            } else if (ret == 0) {
                cout << "Error loading object view!" << endl;
                break;
            }

            // get current view category
            category.str("");
            category.clear();
            category << objectViewRepository.getCategory();

            // training
            if (state == 0) {
                categoryId = memory.addCategory(category.str());
                // learning
            } else {
                categoryId = memory.getCategoryId(category.str());
            }

            if (DEBUG)
                cout << "CATEGORY: " << category.str() << " ID: " << categoryId << endl;

            // keypoints
            PointCloud<PointT>::Ptr keypoints(new PointCloud<PointT>);

            // Get keypoints
            featureExtractor.GetKeypointsUsingAVoxelFilter(pc, voxel_size, keypoints);

            // Spin images
            PointCloud<SpinImage>::Ptr spin_images(new PointCloud<SpinImage>);

            // Compute the keypoints' spin images
            featureExtractor.ComputeSpinImagesAtKeypoints(pc, keypoints,
                                                          search_radius,
                                                          window_width, support_lenght,
                                                          support_angle, min_pts_neighbours, spin_images);

            // Apply memory decay factor. Call before adding new features!
            memory.applyMemoryDecay();

            vector<int> featsIdx;
            char callResult;
            for (size_t i = 0; i < spin_images->points.size(); ++i) {
                // add feature to memory if not redundan
                callResult = memory.addFeature(spin_images->points[i], featId);

                // spin image with nan
                if (callResult == 2 /*|| callResult == 3*/) {
                    continue;
                }


                // keep the ids (map keys) of the features added to memory
                featsIdx.push_back(featId);
            }

            // assign the new features to clusters and do kmeans needed operations
            kMeans.assign(featsIdx);

            // testing phase
            if (state == 1) {
                // get view histogram
                viewHistogram = categoryRecognizer.getViewHistogram(featsIdx/*, redundancyFeatsIdx*/);

                // assign view to a category
                predictedCategoryId = categoryRecognizer.getViewCategory(*viewHistogram);

                // print assignement result
                //if(DEBUG){
                cout << "Category Id:" << endl << "\t"
                     << "Predicted: " << predictedCategoryId
                     << " || Real: " << categoryId << endl;

                cout << "Category:" << endl << "\t"
                     << "Predicted: " << memory.getCategoryName(predictedCategoryId)
                     << " || Real: " << category.str() << endl;
                //}

                // clean up
                delete viewHistogram;

                // assign the object view points to the predicted category
                categoryRecognizer.assignViewToCategory(featsIdx, predictedCategoryId);

                // after each image is tested
                // increment number of samples
                testResult.addToSample();

                if (predictedCategoryId == categoryId) {
                    // if classification was correctly performed increment correct classification count
                    testResult.addToCorrectClassifications();

                    // True Positive
                    memory.addTPToCategory(predictedCategoryId);
                } else {
                    // False Negative
                    memory.addFNToCategory(categoryId);

                    // False Positive
                    memory.addFPToCategory(predictedCategoryId);
                }
            }

            memory.checkRedundancy(featsIdx);
        }
        return memory;
}

/**
 * The application entry point.
 *
 */


    int main(int argc, char **argv) {

        struct timeval timeStart, timeEnd;
        time_t start, end;
        int ss, hh, mm;

        /*
        ss = 2492;
        mm = ss / 60;
        hh = mm / 60;

        cout << int(hh) << ":" << int(mm % 60) << ":" << int(ss % 60) << endl;

        return 1;
        */

        // ./pcd_read views/ 3 65 true false 1 25 false config
        if (argc != 10) {
            cout << "Usage:"
                 << "./pcd_read " << endl
                 << "\t" << "<object_views_directory> " << endl
                 << "\t" << "<num_categories> " << endl
                 << "\t" << "<%_views_4_training> " << endl
                 << "\t" << "<forget: true|false> " << endl
                 << "\t" << "<debug: true|false> " << endl
                 << "\t" << "<number_of_tests> " << endl
                 << "\t" << "<number_of_views_to_see_per_category> " << endl
                 << "\t" << "<use_rand_shuffle: true|false> " << endl
                 << "\t" << "<configuration_file>" << endl
                 << endl;

            return 0;
        }

        gettimeofday(&timeStart, NULL);
        time(&start);

        object_views_directory = argv[1];
        nCategories = atoi(argv[2]);
        trainingPercentage = atoi(argv[3]) / 100.0;
        numberOfTests = atoi(argv[6]);
        nViewsToSeePerCategory = atoi(argv[7]);
        configuration_file = argv[9];

        if (strcmp(argv[4], "true") == 0)
            forget = true;

        if (strcmp(argv[5], "true") == 0)
            DEBUG = true;

        if (strcmp(argv[8], "true") == 0)
            randShuffle = true;

        // load configuration from file
        load_configuration(configuration_file);

        // print configuration
        printConfiguration();

        // Perfomance Manager manages persistence of the performance tests
        PerformanceManager perfManager;



        // run tests
        for (int j = 0; j < numberOfTests; j++) {
            cout << "Test " << (j + 1) << endl;
            cout << "Nº Categories used: " << nCategories << endl;

            // holds the individual test results. A new testResult should be created for each test run

            TestResult *testResult = new TestResult();

            Memory memory = runTest( *testResult);
            cout << "Finisheeeeeeeddddddddddddddd!!!!!!!!!!!!!!!!!!!!!" << endl;
            // run a test


            // collect results
            testResult->setPerfAssess(memory.getCategories());
            testResult->setNumClusters(memory.getNClusters());

            testResult->setNumForgottenFeatures(memory.getNumForgottenFeatures());
            testResult->setNumRedundantFeatures(memory.getNumRedundantFeatures());
            testResult->setNumForgottenFeaturesByMemDecay(
                    memory.getNumForgottenFeaturesByMemoryDecayFactor());
            testResult->setNumSeenObjects(memory.getNumSeenObjects());
            testResult->setNumSeenFeatures(memory.getNumSeenFeatures());
            testResult->setNumFeaturesInMemory(memory.getNumFeaturesInMemory());
            testResult->setSmallestDistance(memory.getSmallestDistance());

            perfManager.addTestResult(testResult);
//
//		// print memory managment information
//		/*
//		cout << "\t" << "Nº forgotten features By Memory Decay Factor: "
//			 << memory.getNumForgottenFeaturesByMemoryDecayFactor()
//			 << endl;
//		*/
//
//		//cout << "\t" << "Max search radius: " << memory.getMaxSearchRadius() << endl;
//		//cout << "\t" << "Min search radius: " << memory.getMinSearchRadius() << endl;
//		cout << "\t" << "Max Global Weight: " << memory.getMaxWg() << endl;
//		cout << "\t" << "Min Global Weigh: " << memory.getMinWg() << endl;
//		cout << "\t" << "Max Local Weight: " << memory.getMaxWl() << endl;
//		cout << "\t" << "Min Local Weigh: " << memory.getMinWl() << endl;
//		cout << "\t" << "Min distance: " << memory.getSmallestDistance() << endl;
//		cout << "\t" << "Max distance: " << memory.getMaxDistance() << endl;
//
//		// print cluster information
            if (DEBUG) {
                cout << "KMeans out " << endl;
                unordered_map<int, Cluster> &clusters = memory.getClusters();

                for (unordered_map<int, Cluster>::iterator clt = clusters.begin(); clt != clusters.end(); ++clt) {
                    // for each cluster
                    Cluster &cluster = clt->second;

                    Histogram<153> &centroid = cluster.getCentroid();

                    //unordered_set<int> &clusterPointsIdx = cluster.getPoints();

                    cout << "cluster " << cluster.getIdentifier() << endl;

                    cout << "cluster centroid " << cluster.getCentroid() << endl;
                }
            }
//
//		/*
//		unordered_map<int, Cluster> &clusters = memory.getClusters();
//		std::string filePath = "out_centers";
//		//ofstream myfile ("example.txt");
//
//		std::ofstream fileStream(filePath.c_str(), std::ifstream::out);
//		if (!fileStream) {
//			cout << "Could not open file " << filePath << endl;
//		}
//		else {
//			for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
//				Histogram<153> &centroid =  clt->second.getCentroid();
//				for(int i = 0; i < 153; ++i) {
//					fileStream << centroid.histogram[i] << " " ;
//				}
//				fileStream << endl;
//			}
//			fileStream.close();
//		}
//
//				unordered_map<int, FeatureMetadata> &feats = memory.getFeatures();
//		std::string ffilePath = "out_feats";
//		//ofstream myfile ("example.txt");
//
//		std::ofstream ffileStream(ffilePath.c_str(), std::ifstream::out);
//		if (!ffileStream) {
//			cout << "Could not open file " << ffilePath << endl;
//		}
//		else {
//			for(unordered_map<int, FeatureMetadata>::iterator featData=feats.begin(); featData != feats.end(); ++featData) {
//				Histogram<153> &feat = featData->second.getFeature();
//				for(int i = 0; i < 153; ++i) {
//					ffileStream <<  feat.histogram[i] * featData->second.getGlobalWeight() << " " ;
//				}
//				ffileStream << featData->second.getCluster();
//				ffileStream << endl;
//			}
//			ffileStream.close();
//		}
//		*/
        }
//
//    // Show performance measures for all categories
        if (numberOfTests > 0)
            perfManager.printPerfResults();

        // Get the execution time.
        gettimeofday(&timeEnd, NULL);
        time(&end);

        long dif = difftime(end, start);

        ss = dif;
        mm = ss / 60;
        hh = mm / 60;

        cout << "Time Elapsed: "
             << int(hh) << ":"
             << int(mm % 60)
             << ":" << int(ss % 60) << endl;

        int h = dif / 3600;
        int m = (dif % 3600) / 60;
        int s = (dif % 3600) % 60;

        cout << "Time Elapsed: "
             << ((h > 9) ? "" : "0") << h << ":"
             << ((m > 9) ? "" : "0") << m << ":"
             << ((s > 9) ? "" : "0") << s
             << endl;

//    /*
//    unsigned long long int timestamp;
//    timestamp = ((timeEnd.tv_sec - timeStart.tv_sec) * 1000000 + (timeEnd.tv_usec - timeStart.tv_usec));
//
//    cout << dif << " : " << timestamp << endl;
//
//    long milliseconds = (long) (timestamp / 1000) % 1000;
//    long seconds = (((long) (timestamp / 1000) - milliseconds) / 1000) % 60;
//    long minutes = (((((long) (timestamp / 1000) - milliseconds)/1000) - seconds)/60) % 60;
//    long hours = ((((((long) (timestamp / 1000) - milliseconds)/1000) - seconds)/60) - minutes)/60;
//
//    cout << "Time Elapsed: "
//	 << ((hours > 9) ? "" : "0") << hours << ":"
//	 << ((minutes > 9) ? "" : "0") << minutes << ":"
//	 << ((seconds > 9) ? "" : "0") << seconds
//	 << endl;
        //   */
        return (0);

}
