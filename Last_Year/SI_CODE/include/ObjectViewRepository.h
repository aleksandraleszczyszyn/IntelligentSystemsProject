#ifndef SI_OBJECTVIEWREPOSITORY_H
#define SI_OBJECTVIEWREPOSITORY_H

#include <include.h>

/**
 * \brief Loads point clouds from a given directory. Also provides shuffle for random point cloud viewing.
 * This class should not be reused because some paremeter are shared and must be re-initialized properly!
 *
 *
 */
class ObjectViewRepository {

	public:
		/**
		 * DEBUG indicates if extra information should be printed to the console.
		 *
		 * @param[in] DEBUG set debug mode on|off.
		 */		
		ObjectViewRepository(bool DEBUG, int nViewsToSeePerCategory, bool randShuffle);

		~ObjectViewRepository();

		/**
		 * Sets the directory containing the object views.
		 *
		 * @param[in] directory the object views directory on disk.
		 */
		char SetObjectViewRepositorySource(const char *directory);

		/**
		 * Returns the next object view to be seen.
		 *
		 * @param[out] pointcloud the next point cloud to be seen.
		 */
		char NextObjectView(PointCloud<PointT>::Ptr& pointcloud);

		/**
		 * Returns the category of the current object view taken from its file name.
		 *
		 * returns the category name of the current object view.
		 */
		string getCategory() const;

		/**
		 * Shuffles views for the training phase. Categories and views are ramdomlly chosen.
		 * The shuffle takes into consideration:
		 *	 the number of categories
		 *	 and the percentage of views from each category, to be use.
		 *
		 * @param[in] nCategories			the number of categories to use
		 * @param[in] percentageForTraining	a percentage of the total number of views available 
		 *									for each category
		 *
		 * assumptions:
		 *		each category has the same, or some equal, number of views available.
		 * 
		 * The shuffle is different for each executation of the application!
		 */
		void shuffleViewsForTraining(int nCategories, float percentageForTraining);

		/**
		 * Shuffles views for the learning phase taking into consideration the views and
		 * categories shuffled for the training phase. No repetations are allowed!
		 *
		 */
		void shuffleViewsForLearning();

		/**
		 * Print the list of available categories, for debug.
		 *
		 */
		void printCategories() const;

		/**
		 * Print the list of available categories and respective views, for debug.
		 *
		 */
		void printCategoryViews() const;

	private:
		/**
		 * Load the categories and respective views from disk.
		 *
		 * @param[in] dir the current directory being scanned for views
		 * @param[in] cat the current category name
		 */
		char LoadRepository(const char* dir, const char* cat);

		/**
		 * Structure to hold the available categories and respective views ids for shuffle.
		 */
		struct CategoryViewIdxs {
		    int categoryIdx;
		    int viewIdx;
		    
		    CategoryViewIdxs(int categoryIdx, int viewIdx) 
				: 
				categoryIdx(categoryIdx),
				viewIdx(viewIdx)
		    {}
		};


		bool randShuffle;

		/// the number of views to see per category
		int nViewsToSeePerCategory;

		/// the category id of the current view being showned
		int currentCategoryId;

		/// holds the views
		vector<CategoryViewIdxs> views;

		/// views repository path on disk
		string object_view_repository;

		/// indicates if class is properly initialized
		bool initialized;

		/// the available categories and respective views
		map<string, vector<string> > objectsMap;

		/// list of categories
		vector<string> categories;

		/// number of categories available on disk
		int nCategoriesAvailable;

		/// number of views per category available on disk
		int nViewsPerCategory;

		/// number of categories requested by the user|application
		int nCategoriesRequested;

		/// percentage of views to be used for training
		float percentageRequested;	

		/// categories shuffled ids
		vector<int> categoriesShuffle;

		/// views shuffled ids
		vector<int> viewsShuffle;

		/// shuffled ids to show
		vector<int> proccessShuffle;

		int viewIdx;

		/// number of views to show
		int nViewsToProccess;

		bool DEBUG;
};

#endif
