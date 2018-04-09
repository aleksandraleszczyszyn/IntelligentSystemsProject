
#include <include.h>
#include <Viewer.h>
#include <Searcher.h>
#include <FeatureExtractor.h>
#include <ObjectViewRepository.h>

//spin images support lenght
float search_radius = 0.1;	//0.1 meters
//0.015 or 0.03			meters
float voxel_size = 0.03;
// spin images parameters
int window_width = 8;
float support_lenght = 0.1;	// 0.1 meters
float support_angle = 0;	// value in [0, 1], related to the cosine of the angle cos(90º) = 0
float min_pts_neighbours = 0;
int K = 3;

bool load_configuration(const char* configuration_file)
{
    ifstream fin(configuration_file);
    string line;
    istringstream sin;

    while (getline(fin, line)) {
 	sin.str(line.substr(line.find("=")+1));

 	if (line.find("search_radius") != string::npos) {
		sin >> search_radius;
		cout << "search_radius " << search_radius << endl;
 	}
 	else if (line.find("voxel_size") != string::npos) {
  		sin >> voxel_size;
		cout << "voxel_size " << voxel_size << endl;
 	}
 	else if (line.find("window_width") != string::npos) {
  		sin >> window_width;
		cout << "window_width " << window_width << endl;
 	}
 	else if (line.find("support_lenght") != string::npos) {
  		sin >> support_lenght;
		cout << "support_lenght " << support_lenght << endl;
 	}
 	else if (line.find("support_angle") != string::npos) {
  		sin >> support_angle;
		cout << "support_angle " << support_angle << endl;
 	}
 	else if (line.find("min_pts_neighbours") != string::npos) {
  		sin >> min_pts_neighbours;
		cout << "min_pts_neighbours " << min_pts_neighbours << endl;
 	}
 	else if (line.find("K") != string::npos) {
  		sin >> K;
		cout << "K " << K << endl;
 	}

 	sin.clear();
    }
}

int main (int argc, char *argv[])
{
    // ./viz /home/u/pcl/SI_CODE/views/apple/apple_1_1_1.pcd config
    if (argc != 3)
    {
	cout << "Usage:"
	     << "./viz "
             << "point_cloud_path "
             << "<configuration_file>" << endl;

	return 0;
    }

    const char* pc_path = argv[1];
    const char* configuration_file = argv[2];

    load_configuration(configuration_file);

    double bin_size = search_radius / window_width / sqrt(2.0);

    const float r = search_radius * window_width;

    int nMatches;

    cout << "bin_size " << bin_size << endl;
    cout << "cylinder r = " << r << " height = " << 2 * r << endl;
    cout << endl;

    Viewer viz = Viewer();
    FeatureExtractor featureExtractor = FeatureExtractor();
    Searcher searcher = Searcher();

    PointCloud<PointT>::Ptr pc (new PointCloud<PointT>);

    if (loadPCDFile(pc_path, *pc) < 0){
	cout << "Object view not found." << endl;
        return 0;
    }

    // keypoints
    PointCloud<PointT>::Ptr keypoints (new PointCloud<PointT>);

    // Get keypoints
    featureExtractor.GetKeypointsUsingAVoxelFilter(pc, voxel_size, keypoints);

    // debug info
    cout << "Nº points: " << pc->points.size() << endl;
    cout << "Nº keypoints: " << keypoints->points.size() << endl;

    // View original point cloud side by side with the keypoints
    viz.VisualizeSideBySide(pc, keypoints);

    // View keypoints on the original point cloud
    viz.VisualizeKeypointsInPointCloud(pc, keypoints);

    // Spin images
    PointCloud<SpinImage >::Ptr spin_images(new PointCloud<SpinImage >);

    // Compute the keypoints' spin images
    featureExtractor.ComputeSpinImagesAtKeypoints(pc, keypoints, search_radius, 
		window_width, support_lenght, support_angle, min_pts_neighbours,
		spin_images);

    // Show some spin image histogram: spin image at index 29 on the spin images point cloud
    viz.ViewSpinImageHistogram(spin_images->points[0]);

    // manipulation tests
    SpinImage &spinImage = spin_images->at(0);

    cout << endl;
    cout << "KDtree search test" << endl;
    cout << endl;

    cout << "3 neareast spin images from spin image 3:" << endl;
    cout << endl;

    vector<int> pointIdxNKNSearch(K);
    vector<float> pointNKNSquaredDistance(K);
    

    nMatches = searcher.KDtreeSPMatch(spin_images->at(3)
			    , spin_images, K
			    , pointIdxNKNSearch
			    , pointNKNSquaredDistance);

    for(int i = 0; i < pointIdxNKNSearch.size(); i++)
    {
	cout << "SP idx: " << pointIdxNKNSearch.at(i) 
	     << "\t distance: " << pointNKNSquaredDistance.at(i) << endl;
    }

    cout << endl;
    cout << "Done!" << endl;
    return 0;
}
