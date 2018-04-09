#include "Category.h"

using namespace std;
    
Category::Category()
{
    throw(std::runtime_error("UNEXPECTED CALL TO Category DEFAULT CONSTRUCTOR"));
}

Category::Category(int identifier, string &name)
    : identifier(identifier), name(name) 
{}

std::string Category::toString() const
{
    ostringstream sstream;
    sstream << name << " Id: " << identifier << " P: ";

    int i = 0;
    for(unordered_set<int>::const_iterator ptIdx = points.begin(); ptIdx != points.end(); ++ptIdx) {
	sstream << *ptIdx;

	if(i != points.size() - 1)
	    sstream << ", ";

        i++;
    }

    return sstream.str();
}

void Category::addPoint(int idx) 
{
    points.insert(idx);
}

void Category::removePoint(int idx) 
{
    points.erase(idx);
}

void Category::getPoints(vector<int> &pointsInd) const 
{
    pointsInd.resize(points.size());
    copy(points.begin(), points.end(), pointsInd.begin());
}
