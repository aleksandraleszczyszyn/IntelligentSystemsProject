#ifndef __CATEGORYRECOGNIZER_H__
#define __CATEGORYRECOGNIZER_H__

#include <map>
#include <stdexcept>

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/common/point_tests.h>

#include "FeatureMetadata.h"
#include "Cluster.h"
#include "Searcher.h"
#include "Memory.h"

#include <tr1/unordered_set>

using std::tr1::unordered_set;

#include <tr1/unordered_map>

using std::tr1::unordered_map;

/**
 * \brief This class assigns an object view to a category, based on the euclidean distance between
 * the object view histogram and the categories histgrams.
 *
 * Histograms are vectors in N dimensions, N = number of clusters.
 * Histograms are computed based on the features' local weight, i.e., its' category weight.
 * Histograms are normalized using:
 *		sqrt(sum(Wli * Wli)) | 
 *			Wli is the sum of the local wieghts of all features that belong to the view|category
 *			and to cluster i
 *			i = 1,..., N, N = number of clusters.
 */
class CategoryRecognizer {
    
    public:
		/**
		 * Creates a new CategoryRecognizer class.
		 *
		 * @param[in] memory reference to the applications' memory structure.
		 */
    	CategoryRecognizer(Memory& memory, bool debugMode = false);

    	~CategoryRecognizer();

		/**
		 * Assigns every feature of a given object view to the given category id.
		 *
		 * @param[in] featsIdx the objects' view feature indeces to assign.
		 * @param[in] categoryId the category id to assign the features to.
		 *
		 */
		void assignViewToCategory(std::vector<int>& featsIdx, int categoryId);

		/**
		 * Computes the objects' view histogram.
		 *
		 * @param[in] featsIdx the objects' view feature indices seen for the first time.
		 * @param[in] redundancyFeatsIdx the objects' view feature indeces of features already in memory (redundancy).						 replacing seen redundant features
		 *
		 * @param[out] returns the view histogram as a vector in N dimensions, 
					   where N is the number of clusters.
		 */
		// @deprecated
		std::vector<double>* getViewHistogram(
							  std::vector<int>& featsIdx, 
							  std::vector<int>& redundancyFeatsIdx);

		/**
		 * Computes the objects' view histogram.
		 *
		 * @param[in] featsIdx the objects' view features indices seen for the first time.
		 *
		 * @param[out] returns the view histogram as a vector in N dimensions, 
					   where N is the number of clusters.
		 */
		std::vector<double>* getViewHistogram(std::vector<int>& featsIdx);

		/**
		 * Assigns a view to a category based on the view and category
		 * histograms. The histograms are normalized!
		 *
		 * Open issues:
		 *		the categories's histograms are computed every time.
		 *
		 * @param[in] viewHistogram the objects' view histogram
		 * @param[out] returns the object view predicted category id.
		 */
		int getViewCategory(std::vector<double>& viewHistogram);

    private:
		/**
		 * Auxiliar structure to hold category-view distances.
		 * It has a notion of distance order using the < operator.
		 *
		 */
		struct CategoryViewDistance{
			int categoryId;
			double distance;

			CategoryViewDistance(int categoryId, double distance)
				: categoryId(categoryId), distance(distance){}

			bool operator < (const CategoryViewDistance& o) const
			{
				return (distance < o.distance);
			}
		};

		/// Reference to the memory structure.
		Memory& memory;

		/// Reference to the searcher used for distance calculations.
		Searcher searcher;
		
		bool DEBUG;
};
    
#endif  // __CATEGORYRECOGNIZER_H__
