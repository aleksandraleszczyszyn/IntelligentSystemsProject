#include <ObjectViewRepository.h>

// random generator function.
int myrandom (int i) {return std::rand() / (1.0 + RAND_MAX) * i;}

ObjectViewRepository::ObjectViewRepository(bool DEBUG, int nViewsToSeePerCategory, bool randShuffle)
	:
	initialized(false),
	nCategoriesAvailable(0),
	nViewsPerCategory(0),			// Not used anymore. Remove?
	DEBUG(DEBUG),
	nViewsToSeePerCategory(nViewsToSeePerCategory),
	randShuffle(randShuffle)
{}

ObjectViewRepository::~ObjectViewRepository()
{}

string ObjectViewRepository::getCategory() const
{
	return categories.at(currentCategoryId);
}

void ObjectViewRepository::shuffleViewsForLearning()
{
	views.erase(views.begin(), views.end());

	// shuffle 
	for(size_t i = 0; i < nCategoriesRequested; i++){
	    for(size_t j = nViewsToProccess; j < nViewsToSeePerCategory; j++){
		views.push_back(CategoryViewIdxs(
			categoriesShuffle.at(i),
			viewsShuffle.at(j)
		));
	    }
	}	

	// nViewsToProccess has a wrong meaning here!
	// nViewsToProccess means the number of views processed in the training phase
	if(views.size() != nCategoriesRequested * (nViewsToSeePerCategory - nViewsToProccess)){
	    throw(std::runtime_error("shuffleViewsForLearning(). Size assertion failled!"));
	}

	proccessShuffle.erase(proccessShuffle.begin(), proccessShuffle.end());

	for(size_t i = 0; i < views.size(); i++){
	    proccessShuffle.push_back(i);
	}
	
	if(randShuffle)
	    std::random_shuffle(proccessShuffle.begin(), proccessShuffle.end(), myrandom);
	else
	    std::random_shuffle(proccessShuffle.begin(), proccessShuffle.end());

	if(DEBUG){
		cout << endl << "Shuffled ids:" << endl;
		for(size_t i = 0; i < proccessShuffle.size(); i++)
		    cout << proccessShuffle[i] << " ";

		cout << endl;
	}

	viewIdx = 0;	
}

void ObjectViewRepository::shuffleViewsForTraining(int nCategories, float percentageForTraining)
{
	if(nCategories > nCategoriesAvailable){
	    cout << "NUMBER CATEGORIES AVAILABLE: " << nCategoriesAvailable << endl;
	    throw(std::runtime_error("Not enough categories available!"));
	}

	if(randShuffle)
	    std::srand(unsigned(std::time(0)));

	percentageRequested = percentageForTraining;
	nCategoriesRequested = nCategories;

	// shuffle categories
	for(size_t i = 0; i < nCategoriesAvailable; i++){
	    categoriesShuffle.push_back(i);
	}

	cout << "Nº Categories Available: " << nCategoriesAvailable << endl;

	//std::random_shuffle(categoriesShuffle.begin(), categoriesShuffle.end(), myrandom);
	if(randShuffle)
	    std::random_shuffle(categoriesShuffle.begin(), categoriesShuffle.end(), myrandom);
	else
	    std::random_shuffle(categoriesShuffle.begin(), categoriesShuffle.end());

	if(DEBUG){
		cout << "Shuffle for categories ids:" << endl;
		for(size_t i = 0; i < categoriesShuffle.size(); i++)
		    cout << categoriesShuffle[i] << " ";

		cout << endl;
	}

	// shuffle wiews
	// assuming all categories have the same number of views
    	map<string, vector<string> >::const_iterator it = objectsMap.begin();

	//nViewsPerCategory = it->second.size();
	//nViewsPerCategory = nViewsToSeePerCategory;

	cout << "Nº Views per Categories: " << nViewsToSeePerCategory << endl;

	for(size_t i = 0; i < nViewsToSeePerCategory; i++){
	    viewsShuffle.push_back(i);
	}
	
	//std::random_shuffle(viewsShuffle.begin(), viewsShuffle.end(), myrandom);
	if(randShuffle)
	    std::random_shuffle(viewsShuffle.begin(), viewsShuffle.end(), myrandom);
	else
	    std::random_shuffle(viewsShuffle.begin(), viewsShuffle.end());

	if(DEBUG){
		cout << "Shuffle for views ids:" << endl;
		for(size_t i = 0; i < viewsShuffle.size(); i++)
		    cout << viewsShuffle[i] << " ";

		cout << endl;
	}

	nViewsToProccess = (int) nViewsToSeePerCategory * percentageForTraining;

	cout << "Nº Views for Training: " << nViewsToProccess << endl;
	cout << "Nº Views for Testing: " << nViewsToSeePerCategory - nViewsToProccess << endl;

	for(size_t i = 0; i < nCategoriesRequested; i++){
	    for(size_t j = 0; j < nViewsToProccess; j++){
		views.push_back(CategoryViewIdxs(
			categoriesShuffle.at(i),
			viewsShuffle.at(j)
		));
	    }
	}

	if(views.size() != nCategoriesRequested * nViewsToProccess){
	    throw(std::runtime_error("shuffleViewsForTraining(). Size assertion failled!"));
	}

	for(size_t i = 0; i < views.size(); i++){
	    proccessShuffle.push_back(i);
	}
	
	//std::random_shuffle(proccessShuffle.begin(), proccessShuffle.end(), myrandom);
	if(randShuffle)
	    std::random_shuffle(proccessShuffle.begin(), proccessShuffle.end(), myrandom);
	else
	    std::random_shuffle(proccessShuffle.begin(), proccessShuffle.end());

	if(DEBUG){
		cout << "Shuffle for category-View ids:" << endl;
		for(size_t i = 0; i < proccessShuffle.size(); i++)
		    cout << proccessShuffle[i] << " ";

		cout << endl;
	}

	viewIdx = 0;
}

void ObjectViewRepository::printCategories() const
{
    cout << "List of available categories:" << endl;

    for(size_t i = 0; i < categories.size(); i++)
    {
	cout << "\t" << categories.at(i) << endl;
    }
}

void ObjectViewRepository::printCategoryViews() const
{
    std::map<string, vector<string> >::const_iterator it = objectsMap.begin();
    std::map<string, vector<string> >::const_iterator itE = objectsMap.end();

    for(; it != itE; ++it)
    {
	cout << endl;
	cout << it->first << endl;

    	for(size_t i = 0; i < it->second.size(); i++)
    	{
	    cout << "\t" << it->second.at(i) << endl;
    	}
    }
}

char ObjectViewRepository::NextObjectView(PointCloud<PointT>::Ptr& pointcloud)
{
    if(!initialized)
	return 3;

    if(viewIdx >= views.size())
	return 2;

    const CategoryViewIdxs& categoryViewIdxs = views.at(proccessShuffle.at(viewIdx));
    string category(categories.at(categoryViewIdxs.categoryIdx));

    currentCategoryId = categoryViewIdxs.categoryIdx;

    string path;
    path.append(object_view_repository);
    path.append(category);
    path.append("/");
    path.append(objectsMap[category].at(categoryViewIdxs.viewIdx));

    //if(DEBUG)
        cout << "View: " << path << endl;

    if (loadPCDFile (path, *pointcloud) < 0)
	return 0;

    viewIdx++;

    return 1;
}

char ObjectViewRepository::SetObjectViewRepositorySource(const char *directory)
{
    object_view_repository = directory;
    return LoadRepository(directory, "");
}

char ObjectViewRepository::LoadRepository(const char* dir, const char* cat)
{
    int fd, nread;
    char buf[BUF_SIZE];
    struct linux_dirent *d;
    int bpos;
    char d_type;
    
    fd = open(dir, O_RDONLY | O_DIRECTORY);

    if (fd == -1){
	return 2;
    }

    for ( ; ; ) {
        nread = syscall(SYS_getdents, fd, buf, BUF_SIZE);

        if (nread == -1){
	    return 3;
	}

       if (nread == 0)
            break;

        for (bpos = 0; bpos < nread;) {
            d = (struct linux_dirent *) (buf + bpos);

	    d_type = *(buf + bpos + d->d_reclen - 1);
	    
	    // view file
	    if(d_type == DT_REG){
    		std::map<string, vector<string> >::iterator cv = objectsMap.find(cat);

    		if(cv == objectsMap.end()) {
        		std::cout << "Unexpected Error: no category found for " << d->d_name  << '\n';
			return 4;	
    		}
    		else {
                        vector<string>& categoryViews = cv->second;
			categoryViews.push_back(string(d->d_name));
   		}		
	    }
	    // category directory
	    else if(d_type == DT_DIR){
		if(strcmp(d->d_name, ".") != 0 && strcmp(d->d_name, "..") != 0)
		{
			vector<string> views;
			objectsMap[d->d_name] = views;

			string category(d->d_name);
			categories.push_back(category);
			nCategoriesAvailable++;

 			string path(dir);
			path.append("/");
			path.append(d->d_name);

			LoadRepository(path.c_str(), d->d_name);
		}
	    }
		
            bpos += d->d_reclen;
        }
    }

    initialized = true;
    return 1;
}
