#ifndef SI_FEATUREEXTRACTOR_H
#define SI_FEATUREEXTRACTOR_H

#include <include.h>

/**
 * \brief Functions to extract keypoints from a point cloud and compute spin images.
 *
 * In this version the keypoints are extracted using a voxel grid filter.
 */
class FeatureExtractor {

	public:
		FeatureExtractor();

		~FeatureExtractor();

		/**
		 * Find keypoints using a Voxel Grid Filter.
		 *
		 * @param[in] pc			the point cloud to get keypoints from
		 * @param[in] voxelSize		the voxel grid filter size
		 * @param[out] keypoints	the keypoints found using the Voxel Grid Filter
	 	 *
		 */
		void GetKeypointsUsingAVoxelFilter(
				const PointCloud<PointT>::Ptr& pc, 
				float voxelSize, 
				PointCloud<PointT>::Ptr& keypoints) const;

		/**
		 * Computes spin images for a set of given points.
		 *
		 * @param[in] search_surface	the search surface
		 * @param[in] keypoints			the keypoints, points, to get the spin images for
		 * @param[in] radius_search		paremeters for spin image generation
		 * @param[in] window_width
		 * @param[in] support_lenght
		 * @param[in] support_angle
		 * @param[in] min_pts_neighbours
		 * @param[out] spin_images		the computed spin images for each point
		 */		
		void ComputeSpinImagesAtKeypoints(
				const PointCloud<PointT>::Ptr& search_surface, 
                const PointCloud<PointT>::Ptr& keypoints, 
				float radius_search,
				int window_width,
    			float support_lenght,
    			float support_angle,
    			float min_pts_neighbours,
                PointCloud<SpinImage >::Ptr& spin_images) const;

		/**
		 * Computes the surface normals.
		 *
		 * @param[in] pc				the point cloud
		 * @param[in] normal_radius		paremeter for normals computation
		 * @param[out] pc_normals_out	the point cloud computed normals
		 */	
		void ComputeSurfaceNormals(
				const PointCloud<PointT>::Ptr &pc, 
				float normal_radius,
                PointCloud<Normal>::Ptr &pc_normals_out) const;
};

#endif
