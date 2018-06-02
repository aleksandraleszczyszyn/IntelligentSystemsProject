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

#include <boost/thread.hpp>

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
const int numberOfThreads = 3;
const int numIters = 10;
bool featureFinished = false;

int initialNumberOfClusters = 7;

int minNumberOfClusters = initialNumberOfClusters;
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
int trainingPhase(Memory& memory, TestResult& testResult, FeatureExtractor* featureExtractor, ObjectViewRepository* objectViewRepository, KMeans* kMeans, float& wss, float& bss, int iteration)
{

    ostringstream category;
    int categoryId;
    int featId;
    char ret;
    char callResult;
    int counter = 0;
    while(1)
    {
        PointCloud<PointT>::Ptr pc (new PointCloud<PointT>);
        ret = objectViewRepository->NextObjectView(pc);
        counter++;
        if(counter%(10*iteration) == 0){
            return 0;
        }
        if(ret == 3)
        {
            cout << "The ObjectViewRepository is not initialized yet!" << endl;
            break;
        }

        else if(ret == 2)
        {     //if we reach end of dataset
                cout << endl;
                cout << "Training done!" << endl;
                cout << endl;

                // learning
                cout << "Starting Testing phase!" << endl;
                objectViewRepository->shuffleViewsForLearning();
                featureFinished = true;
                return 1;

        }
        else if(ret == 5)
        {
            cout << "No enough categories available!" << endl;
            break;
        }
        else if(ret == 0)
        {
            cout << "Error loading object view!" << endl;
            break;
        }

        // get current view category
        category.str("");
        category.clear();
        category << objectViewRepository->getCategory();

        categoryId = memory.addCategory(category.str());

        if(DEBUG)
            cout << "CATEGORY: " << category.str() << " ID: " << categoryId << endl;

        // keypoints
        PointCloud<PointT>::Ptr keypoints (new PointCloud<PointT>);

        // Get keypoints
        featureExtractor->GetKeypointsUsingAVoxelFilter(pc, voxel_size, keypoints);

        // Spin images
        PointCloud<SpinImage >::Ptr spin_images(new PointCloud<SpinImage >);

        // Compute the keypoints' spin images
        featureExtractor->ComputeSpinImagesAtKeypoints(pc, keypoints,
                                                      search_radius,
                                                      window_width, support_lenght,
                                                      support_angle, min_pts_neighbours, spin_images);

        // Apply memory decay factor. Call before adding new features!
        memory.applyMemoryDecay();

        vector<int> featsIdx; //creating features ids vector for one object view

        for (size_t i = 0; i < spin_images->points.size(); ++i) {
            // add feature to memory if not redundant
            callResult = memory.addFeature(spin_images->points[i], featId);

            // spin image with nan
            if(callResult == 2 /*|| callResult == 3*/) {
                continue;
            }

            // if training, add feature to category
            memory.addPointToCategory(categoryId, featId);


            // keep the ids (map keys) of the features added to memory
            featsIdx.push_back(featId);
        }

        // assign the new features to clusters and do kmeans needed operations
        kMeans->assign(featsIdx);

        memory.checkRedundancy(featsIdx);
    }
}

void testingPhase(Memory& memory, TestResult& testResult, FeatureExtractor* featureExtractor, ObjectViewRepository* objectViewRepository, KMeans* kMeans, CategoryRecognizer* categoryRecognizer)
{
    ostringstream category;
    vector<double>* viewHistogram;
    int categoryId;
    int predictedCategoryId;
    int featId;
    char ret;
    char callResult;

    while(1)
    {
            PointCloud<PointT>::Ptr pc (new PointCloud<PointT>);
            ret = objectViewRepository->NextObjectView(pc);

            if(ret == 3){
                cout << "The ObjectViewRepository is not initialized yet!" << endl;
                break;
            }
            else if(ret == 2)
            {     //if we reach end of dataset
                cout << endl << "Testing done!" << endl;
                break;
            }
            else if(ret == 5)
            {
                cout << "No enough categories available!" << endl;
                break;
            }
            else if(ret == 0)
            {
                cout << "Error loading object view!" << endl;
                break;
            }

            // get current view category
            category.str("");
            category.clear();
            category << objectViewRepository->getCategory();

            // training
            categoryId = memory.getCategoryId(category.str());

            if(DEBUG)
                cout << "CATEGORY: " << category.str() << " ID: " << categoryId << endl;

            // keypoints
            PointCloud<PointT>::Ptr keypoints (new PointCloud<PointT>);

            // Get keypoints
            featureExtractor->GetKeypointsUsingAVoxelFilter(pc, voxel_size, keypoints);

            // Spin images
            PointCloud<SpinImage >::Ptr spin_images(new PointCloud<SpinImage >);

            // Compute the keypoints' spin images
            featureExtractor->ComputeSpinImagesAtKeypoints(pc, keypoints,
                                                          search_radius,
                                                          window_width, support_lenght,
                                                          support_angle, min_pts_neighbours, spin_images);

            // Apply memory decay factor. Call before adding new features!
            memory.applyMemoryDecay();

            vector<int> featsIdx; //creating features ids vector for one object view

            for (size_t i = 0; i < spin_images->points.size(); ++i) {
                // add feature to memory if not redundant
                callResult = memory.addFeature(spin_images->points[i], featId);

                // spin image with nan
                if(callResult == 2 /*|| callResult == 3*/) {
                    continue;
                }

                // keep the ids (map keys) of the features added to memory
                featsIdx.push_back(featId);
            }

            // assign the new features to clusters and do kmeans needed operations
            kMeans->assign(featsIdx);

            // learning phase
                // get view histogram
                viewHistogram = categoryRecognizer->getViewHistogram(featsIdx/*, redundancyFeatsIdx*/);

                // assign view to a category
                predictedCategoryId = categoryRecognizer->getViewCategory(*viewHistogram);

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
                categoryRecognizer->assignViewToCategory(featsIdx, predictedCategoryId);

                // after each image is tested
                // increment number of samples
                testResult.addToSample();

                if (predictedCategoryId == categoryId) {
                    // if classification was correctly performed increment correct classification count
                    testResult.addToCorrectClassifications();

                    // True Positive
                    memory.addTPToCategory(predictedCategoryId);
                }
                else {
                    // False Negative
                    memory.addFNToCategory(categoryId);

                    // False Positive
                    memory.addFPToCategory(predictedCategoryId);
                }
            memory.checkRedundancy(featsIdx);
    }
}

class ThreadSctructures {
public:
    Memory memory;
    TestResult* testResult;
    FeatureExtractor featureExtractor;
    ObjectViewRepository objectViewRepository;
    KMeans kMeans;
    CategoryRecognizer categoryRecognizer;
    float wss;
    float bss;
    int bestThreadIndex;
    int iteration=1;

    ThreadSctructures(): categoryRecognizer(memory, DEBUG), kMeans(numIters), objectViewRepository(DEBUG, nViewsToSeePerCategory, randShuffle)
    {
        objectViewRepository.SetObjectViewRepositorySource(object_views_directory);
        objectViewRepository.shuffleViewsForTraining(nCategories, trainingPercentage);
        memory.setMemoryDecayFactor(memoryDecayFactor);
        memory.setFeatureRedundancyTreshold(featureRedundancyTreshold);
        memory.setMaxNumFeaturesInMemory(maxNumFeaturesInMemory);
        memory.setMemoryDecayTreshold(memoryDecayTreshold);
        memory.setDebugMode(DEBUG);
        memory.setForgetFeatures(forget);
        memory.setGWRedistSearchRadius(redistribute_gWeights_search_radius);
        featureExtractor = FeatureExtractor();
        testResult = new TestResult();
    }

    void setNumberOfClusters(int number)
    {
        kMeans.setNumberOfClusters(number);
        kMeans.init(memory);
        memory.setKmeans(&kMeans);

        cout<<"Kmeans cluster: "<<kMeans.numOfClusters<<" Memory n cluster: "<<memory.getNClusters()<<"   BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"<<endl;
    }

    void ReduceNumberOfClusters() {
        if (minNumberOfClusters > 2) {
            minNumberOfClusters--;
            cout << "Decreasing number of clusters !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "
                 << kMeans.numOfClusters << endl;
        }
    }

    void IncreaseNumberOfClusters()
    {
        minNumberOfClusters++;
        memory.setKmeans(&kMeans);
        cout << "Increasing number of clusters !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "
             << kMeans.numOfClusters << endl;
    }
};


char runTest(ThreadSctructures threadSctructures[]){

    ObjectViewRepository objectViewRepository = ObjectViewRepository(DEBUG, nViewsToSeePerCategory, randShuffle);

    char callResult = objectViewRepository.SetObjectViewRepositorySource(object_views_directory);
    if(callResult == 2)
    {
        cout << "Error loading the specified views repository." << endl;
        return 0;
    }
    else if(callResult == 3)
    {
        cout << "Error reading file/diretory in the views repository." << endl;
        return 0;
    }
    else if(callResult == 4)
    {
        return 0;
    }

    ostringstream category;

    cout << endl << "Task: Training" << endl << endl;

    thread *threads[numberOfThreads];

    while(!featureFinished){

        //TODO check results of previous iteration if there is previous
        cout<<"While loop iteration!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

        for (int i=0; i<numberOfThreads; i++)
        {
            threadSctructures[i].memory = Memory();
        }

        // set the number of clusters for each thread
        for (int i=0; i<numberOfThreads; i++)
        {
            threadSctructures[i].setNumberOfClusters(minNumberOfClusters+i);
        }
        for (int i=0; i<numberOfThreads; i++) {
            threads[i] = new thread(trainingPhase, boost::ref(threadSctructures[i].memory),
                                    *threadSctructures[i].testResult, &threadSctructures[i].featureExtractor,
                                    &threadSctructures[i].objectViewRepository, &threadSctructures[i].kMeans,
                                    boost::ref(threadSctructures[i].wss), boost::ref(threadSctructures[i].bss),
                                    threadSctructures[i].iteration);

            threadSctructures[i].iteration++;

        }

        for (int i=0; i<numberOfThreads; i++)
        {
            threads[i]->join();
            delete threads[i];
        }
        // finding the best thread
        int best=0;
        float bestCH = threadSctructures[0].memory.Ch();
        float ch;
        for (int i=0; i<numberOfThreads; i++) {

            ch=threadSctructures[i].memory.Ch();
            cout<<"CH["<<i<<"]= "<<ch<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<endl;
            if (ch > bestCH)
            {
                best = i;
                bestCH = ch;
            }
            threadSctructures[i].bestThreadIndex=best;
        }

        if (numberOfThreads%2==0)
        {
            int middle = numberOfThreads/2;
            if (best < middle-1)
            {

                threadSctructures[0].ReduceNumberOfClusters();

            }

            if (best > middle)
            {

                threadSctructures[0].IncreaseNumberOfClusters();

            }
        } else{


            int middle = numberOfThreads/2;
            cout << "middle: " << middle << " best: " << best << " n of threads: " << numberOfThreads << "      33333333333333333333333333333333333333333333333333"<< endl;
            if (best < middle)
            {
                    threadSctructures[0].ReduceNumberOfClusters();
            }

            if (best>middle)
            {
                threadSctructures[0].IncreaseNumberOfClusters();

            }

        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //// finding the best threads





    //trainingPhase(memory, testResult, &featureExtractor, &objectViewRepository, &kMeans);
//
//    testingPhase(threadSctructures[best].memory,*threadSctructures[best].testResult, &threadSctructures[best].featureExtractor, &threadSctructures[best].objectViewRepository, &threadSctructures[best].kMeans,&threadSctructures[best].categoryRecognizer);
}

/**
 * The application entry point.
 *
 */
int main (int argc, char** argv){

    struct timeval timeStart, timeEnd;
    time_t start,end;
    int ss, hh, mm;

    /*
    ss = 2492;
    mm = ss / 60;
    hh = mm / 60;

    cout << int(hh) << ":" << int(mm % 60) << ":" << int(ss % 60) << endl;

    return 1;
    */

    // ./pcd_read views/ 3 65 true false 1 25 false config
    if (argc != 10)
    {
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
    time (&start);

    object_views_directory = argv[1];
    nCategories = atoi(argv[2]);
    trainingPercentage = atoi(argv[3]) / 100.0;
    numberOfTests = atoi(argv[6]);
    nViewsToSeePerCategory = atoi(argv[7]);
    configuration_file = argv[9];

    if(strcmp(argv[4], "true") == 0)
        forget = true;

    if(strcmp(argv[5], "true") == 0)
        DEBUG = true;

    if(strcmp(argv[8], "true") == 0)
        randShuffle = true;

    // load configuration from file
    load_configuration(configuration_file);

    // print configuration
    printConfiguration();

    // Perfomance Manager manages persistence of the performance tests
    PerformanceManager perfManager;

    // run tests
    for(int j = 0; j < numberOfTests; j++){
        cout << "Test " << (j + 1) << endl;
        cout << "Nº Categories used: " << nCategories << endl;

        // holds the individual test results. A new testResult should be created for each test run
        //TestResult* testResult = new TestResult();


        ThreadSctructures threadSctructures[numberOfThreads];

        // set the number of clusters for each thread
        for (int i=0; i<numberOfThreads; i++)
        {
            threadSctructures[i].setNumberOfClusters(initialNumberOfClusters+i);
        }

        // training phase
        runTest(threadSctructures);
        int best = threadSctructures[0].bestThreadIndex;

        // testing phase

                cout << "Best thread index: " << best << " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
        cout << "Best thread index: " << threadSctructures[best].kMeans.numOfClusters << " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
        testingPhase(threadSctructures[best].memory,*threadSctructures[best].testResult, &threadSctructures[best].featureExtractor, &threadSctructures[best].objectViewRepository, &threadSctructures[best].kMeans,&threadSctructures[best].categoryRecognizer);


        for (int i=0; i<numberOfThreads; i++)
        {
            cout << "Thread " << i << " wss: " << threadSctructures[i].wss << " @@@@@@@@@@@######################################$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
            cout << "Thread " << i << " bss: " << threadSctructures[i].bss << " @@@@@@@@@@@######################################$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
        }

        // collect results
        threadSctructures[best].testResult->setPerfAssess(threadSctructures[best].memory.getCategories());

        threadSctructures[best].testResult->setNumClusters(threadSctructures[best].memory.getNClusters());

        threadSctructures[best].testResult->setNumForgottenFeatures(threadSctructures[best].memory.getNumForgottenFeatures());
        threadSctructures[best].testResult->setNumRedundantFeatures(threadSctructures[best].memory.getNumRedundantFeatures());
        threadSctructures[best].testResult->setNumForgottenFeaturesByMemDecay(
                threadSctructures[best].memory.getNumForgottenFeaturesByMemoryDecayFactor());
        threadSctructures[best].testResult->setNumSeenObjects(threadSctructures[best].memory.getNumSeenObjects());
        threadSctructures[best].testResult->setNumSeenFeatures(threadSctructures[best].memory.getNumSeenFeatures());
        threadSctructures[best].testResult->setNumFeaturesInMemory(threadSctructures[best].memory.getNumFeaturesInMemory());
        threadSctructures[best].testResult->setSmallestDistance(threadSctructures[best].memory.getSmallestDistance());

        perfManager.addTestResult(threadSctructures[best].testResult);

        // print memory managment information
        /*
        cout << "\t" << "Nº forgotten features By Memory Decay Factor: "
             << memory.getNumForgottenFeaturesByMemoryDecayFactor()
             << endl;
        */

        //cout << "\t" << "Max search radius: " << memory.getMaxSearchRadius() << endl;
        //cout << "\t" << "Min search radius: " << memory.getMinSearchRadius() << endl;
        cout << "\t" << "Max Global Weight: " << threadSctructures[best].memory.getMaxWg() << endl;
        cout << "\t" << "Min Global Weigh: " << threadSctructures[best].memory.getMinWg() << endl;
        cout << "\t" << "Max Local Weight: " << threadSctructures[best].memory.getMaxWl() << endl;
        cout << "\t" << "Min Local Weigh: " << threadSctructures[best].memory.getMinWl() << endl;
        cout << "\t" << "Min distance: " << threadSctructures[best].memory.getSmallestDistance() << endl;
        cout << "\t" << "Max distance: " << threadSctructures[best].memory.getMaxDistance() << endl;

        // print cluster information
        if(DEBUG){
            cout << "KMeans out " << endl;
            unordered_map<int, Cluster> &clusters = threadSctructures[best].memory.getClusters();

            for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
                // for each cluster
                Cluster &cluster = clt->second;

                Histogram<153> &centroid = cluster.getCentroid();

                //unordered_set<int> &clusterPointsIdx = cluster.getPoints();

                cout << "cluster " << cluster.getIdentifier() << endl;

                cout << "cluster centroid " << cluster.getCentroid() << endl;
            }
        }

        /*
        unordered_map<int, Cluster> &clusters = memory.getClusters();
        std::string filePath = "out_centers";
        //ofstream myfile ("example.txt");

        std::ofstream fileStream(filePath.c_str(), std::ifstream::out);
        if (!fileStream) {
            cout << "Could not open file " << filePath << endl;
        }
        else {
            for(unordered_map<int, Cluster>::iterator clt=clusters.begin(); clt != clusters.end(); ++clt) {
                Histogram<153> &centroid =  clt->second.getCentroid();
                for(int i = 0; i < 153; ++i) {
                    fileStream << centroid.histogram[i] << " " ;
                }
                fileStream << endl;
            }
            fileStream.close();
        }

                unordered_map<int, FeatureMetadata> &feats = memory.getFeatures();
        std::string ffilePath = "out_feats";
        //ofstream myfile ("example.txt");

        std::ofstream ffileStream(ffilePath.c_str(), std::ifstream::out);
        if (!ffileStream) {
            cout << "Could not open file " << ffilePath << endl;
        }
        else {
            for(unordered_map<int, FeatureMetadata>::iterator featData=feats.begin(); featData != feats.end(); ++featData) {
                Histogram<153> &feat = featData->second.getFeature();
                for(int i = 0; i < 153; ++i) {
                    ffileStream <<  feat.histogram[i] * featData->second.getGlobalWeight() << " " ;
                }
                ffileStream << featData->second.getCluster();
                ffileStream << endl;
            }
            ffileStream.close();
        }
        */
    }

    // Show performance measures for all categories
    if(numberOfTests > 0)
        perfManager.printPerfResults();

    // Get the execution time.
    gettimeofday(&timeEnd, NULL);
    time (&end);

    long dif = difftime (end, start);

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

    /*
    unsigned long long int timestamp;
    timestamp = ((timeEnd.tv_sec - timeStart.tv_sec) * 1000000 + (timeEnd.tv_usec - timeStart.tv_usec));

    cout << dif << " : " << timestamp << endl;

    long milliseconds = (long) (timestamp / 1000) % 1000;
    long seconds = (((long) (timestamp / 1000) - milliseconds) / 1000) % 60;
    long minutes = (((((long) (timestamp / 1000) - milliseconds)/1000) - seconds)/60) % 60;
    long hours = ((((((long) (timestamp / 1000) - milliseconds)/1000) - seconds)/60) - minutes)/60;

    cout << "Time Elapsed: "
	 << ((hours > 9) ? "" : "0") << hours << ":"
	 << ((minutes > 9) ? "" : "0") << minutes << ":"
	 << ((seconds > 9) ? "" : "0") << seconds
	 << endl;
    */
    return (0);
}
