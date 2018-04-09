
#ifndef SI_OBJECTVIEWREPOSITORY_H
#define SI_OBJECTVIEWREPOSITORY_H

#include <include.h>

/**
 * Loads point clouds from a given directory.
 *
 *
 */
class ObjectViewRepository {

	public:
		ObjectViewRepository();

		~ObjectViewRepository();

		char SetObjectViewRepositorySource(const char *directory);

		char SetTrainingRepositorySource(const char *directory);

		char NextObjectView(PointCloud<PointT>::Ptr& pointcloud);

		void printCategories() const;

		void printCategoryViews() const;

	private:
		// load the object views
		char LoadRepository(const char* dir, const char* cat);

		// to learn
		string object_view_repository;

		// for training
		string training_repository;

		int counter;

		vector<string> object_views;

		bool initialized;

		// task to execute (learn|training)
		int task;

		map<string, vector<string> > objectsMap;


		vector<string> categories;
};

#endif
