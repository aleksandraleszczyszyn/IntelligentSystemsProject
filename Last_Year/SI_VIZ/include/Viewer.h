
#ifndef SI_VIEWER_H
#define SI_VIEWER_H

#include <include.h>

/**
 *
 *
 *
 */
class Viewer {

	public:
		Viewer();

		~Viewer();

		void ViewSpinImageHistogram(const SpinImage &descriptor) const;

		void VisualizeKeypointsInPointCloud(const PointCloud<PointT>::Ptr pc,
                          const PointCloud<PointT>::Ptr keypoints) const;

		void VisualizeSideBySide(const PointCloud<PointT>::Ptr leftWindowPC,
                        const PointCloud<PointT>::Ptr rightWindowPC) const;

	private:
};

#endif
