
#ifndef SI_INCLUDE_H
#define SI_INCLUDE_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include<pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/common/common.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/vtk.h>
#include <pcl/features/3dsc.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/pca.h>
#include <pcl/common/impl/pca.hpp>

#include <stdio.h>
#include <sys/types.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

//#define _GNU_SOURCE
#include <dirent.h>     /* Defines DT_* constants */
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/syscall.h>

#include <map>

using namespace pcl::io;
using namespace pcl::visualization;
using namespace pcl;
using namespace std;

typedef Histogram<153> SpinImage;
//typedef PointXYZ PointT;
typedef PointXYZRGB PointT;

struct linux_dirent {
    long           d_ino;
    off_t          d_off;
    unsigned short d_reclen;
    char           d_name[];
};

#define BUF_SIZE 1024

#define LEARNING 1
#define TRAINING 2

using namespace std;


// we need this to compare the spin images with the kd-tree
namespace pcl {
	template <>
	class DefaultPointRepresentation<SpinImage> : public PointRepresentation<SpinImage>
	{
		public:
  			DefaultPointRepresentation ()
  			{
    				nr_dimensions_ = 153;
  			}

  		virtual void copyToFloatArray (const SpinImage &p, float *out) const
  		{
    			for (int i = 0; i < nr_dimensions_; ++i)
      				out[i] = p.histogram[i];
  		}
	};
}
// end

#endif
