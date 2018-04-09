
#include <FeatureExtractor.h>

FeatureExtractor::FeatureExtractor()
{}

FeatureExtractor::~FeatureExtractor()
{}

void FeatureExtractor::GetKeypointsUsingAVoxelFilter(
		const PointCloud<PointT>::Ptr& pc, 
		float voxelSize, PointCloud<PointT>::Ptr& keypoints) const
{
    VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(pc);
    voxelGrid.setLeafSize (voxelSize, voxelSize, voxelSize);
    voxelGrid.filter(*keypoints);
}

void FeatureExtractor::ComputeSpinImagesAtKeypoints(
		const PointCloud<PointT>::Ptr &search_surface, 
                const PointCloud<PointT>::Ptr &keypoints, 
		float radius_search,
		int window_width,
    		float support_lenght,
    		float support_angle,
    		float min_pts_neighbours,
                PointCloud<SpinImage >::Ptr &spin_images) const
{
    SpinImageEstimation<PointT, Normal, SpinImage > spin_image_descriptor(window_width, support_angle, min_pts_neighbours);

    search::KdTree<PointT>::Ptr kdtree (new search::KdTree<PointT>);
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);

    spin_image_descriptor.setSearchSurface(search_surface);

    ComputeSurfaceNormals(keypoints, radius_search, normals);

    spin_image_descriptor.setInputCloud(keypoints);

    spin_image_descriptor.setInputNormals(normals);
    
    spin_image_descriptor.setSearchMethod(kdtree);

    spin_image_descriptor.setRadiusSearch(support_lenght);

    spin_image_descriptor.compute(*spin_images);  
}

void FeatureExtractor::ComputeSurfaceNormals(
		const PointCloud<PointT>::Ptr &pc, 
		float normal_radius,
                PointCloud<Normal>::Ptr &pc_normals_out) const
{
    NormalEstimation<PointT, Normal> norm_est;

    norm_est.setSearchMethod(search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));

    // Specify the size of the local neighborhood to use when computing the surface normals
    norm_est.setRadiusSearch(normal_radius);

    // Set the input pc
    norm_est.setInputCloud(pc);

    // Estimate the surface normals and store the result in "normals_out"
    norm_est.compute(*pc_normals_out);
}
