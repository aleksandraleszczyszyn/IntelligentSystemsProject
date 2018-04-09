
#ifndef SI_VIEWER_H
#define SI_VIEWER_H

#include <include.h>

/**
 * \brief Functions for visualization of points|features and histograms.
 *
 *
 */
class Viewer {

	public:
		Viewer();

		~Viewer();

		/**
		 * View the histogram of a given spin image.
		 *
		 * @param[in] descriptor spin image
		 *
		 */
		void ViewSpinImageHistogram(const SpinImage &descriptor) const;

		/**
		 * View the keypoints in the original point cloud.
		 * Generalization: view a set of points as spheres in a universe of points.
		 *		   Where: set of points << universe of points
		 *
		 * @param[in] pc		the universe of points
		 * @param[in] keypoints	a subset of points of the universe of points
		 */
		void VisualizeKeypointsInPointCloud(
						const PointCloud<PointT>::Ptr pc,
                        const PointCloud<PointT>::Ptr keypoints) const;

		/**
		 * View the keypoints and the original point cloud in two windows side by side.
		 * Generalization: view two sets of points in two windows side by side.
		 *
		 * @param[in] leftWindowPC	point cloud to show in the left window
		 * @param[in] rightWindowPC	point cloud to show in the right window
		 */
		void VisualizeSideBySide(
						const PointCloud<PointT>::Ptr leftWindowPC,
                        const PointCloud<PointT>::Ptr rightWindowPC) const;
};

#endif
