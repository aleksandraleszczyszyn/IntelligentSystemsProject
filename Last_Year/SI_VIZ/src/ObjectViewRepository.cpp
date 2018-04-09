

#include <ObjectViewRepository.h>

ObjectViewRepository::ObjectViewRepository()
	:
	counter(0),
	initialized(false),
	task(0)
{
	//TODO
}

ObjectViewRepository::~ObjectViewRepository()
{
	//TODO
}

void ObjectViewRepository::printCategories() const
{
    cout << endl;
    cout << "List of categories" << endl;

    for(size_t i = 0; i < categories.size(); i++)
    {
	cout << "\t" << categories.at(i) << endl;
    }

    cout << endl;
}

void ObjectViewRepository::printCategoryViews() const
{
    cout << endl;

    std::map<string, vector<string> >::const_iterator it = objectsMap.begin();
    std::map<string, vector<string> >::const_iterator itE = objectsMap.end();

    for(; it != itE; ++it)
    //for(const map<string, vector<string > >::iterator it = objectsMap.begin(); it != objectsMap.end(); ++it)
    {
	cout << endl;
	cout << it->first << endl;

    	for(size_t i = 0; i < it->second.size(); i++)
    	{
	    cout << "\t" << it->second.at(i) << endl;
    	}

    	cout << endl;
    }

    cout << endl;
}

char ObjectViewRepository::NextObjectView(PointCloud<PointT>::Ptr& pointcloud)
{
    if(!initialized)
	return 3;

    if(counter >= object_views.size())
	return 2;

    if (loadPCDFile (object_view_repository + object_views.at(counter), *pointcloud) < 0)
        return 0;

    counter++;
    return 1;
}

char ObjectViewRepository::SetObjectViewRepositorySource(const char *directory)
{
    object_view_repository = directory;
    task = LEARNING;
    return LoadRepository(directory, "");
}

char ObjectViewRepository::SetTrainingRepositorySource(const char *directory)
{
    training_repository = directory;
    task = TRAINING;
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

	    if(d_type == DT_REG){
    		std::map<string, vector<string> >::iterator cv = objectsMap.find(cat);
    		if(cv == objectsMap.end()) {
        		std::cout << "Error: no category found for " << d->d_name  << '\n';
    		}
    		else {
                        vector<string>& categoryViews = cv->second;
			
			string categoryView(d->d_name);
			categoryViews.push_back(categoryView);

			string o(cat);
			o.append("/");
			o.append(d->d_name);
			object_views.push_back(o);
   		}		
	    }
	    else if(d_type == DT_DIR){
		if(strcmp(d->d_name, ".") != 0 && strcmp(d->d_name, "..") != 0)
		{
			vector<string> views;
			objectsMap[d->d_name] = views;

			string category(d->d_name);
			categories.push_back(category);

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

