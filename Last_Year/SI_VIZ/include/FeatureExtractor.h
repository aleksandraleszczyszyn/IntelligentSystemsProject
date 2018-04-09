
#ifndef SI_FEATUREEXTRACTOR_H
#define SI_FEATUREEXTRACTOR_H

#include <include.h>

/**
 *
 *
 *
 */
class FeatureExtractor {

	public:
		FeatureExtractor();

		~FeatureExtractor();

		void GetKeypointsUsingAVoxelFilter(const PointCloud<PointT>::Ptr& pc, float voxelSize, PointCloud<PointT>::Ptr& keypoints) const;

		void ComputeSpinImagesAtKeypoints(PointCloud<PointT>::Ptr& search_surface, 
                                   PointCloud<PointT>::Ptr& keypoints, 
				   float radius_search,
				   int window_width,
    				   float support_lenght,
    				   float support_angle,
    				   float min_pts_neighbours,
                                   PointCloud<SpinImage >::Ptr& spin_images) const;

		void ComputeSurfaceNormals(PointCloud<PointT>::Ptr &pc, 
				   float normal_radius,
                		   PointCloud<Normal>::Ptr &pc_normals_out) const;
};

#endif
