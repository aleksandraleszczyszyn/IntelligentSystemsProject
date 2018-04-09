#include <pcl/point_types.h>

namespace pcl {
	template <>
	class DefaultPointRepresentation<Histogram<153> > : public PointRepresentation<Histogram<153> >
	{
	    public:

	    DefaultPointRepresentation ()
	    {
		nr_dimensions_ = 153;
	    }

	    virtual void
	    copyToFloatArray (const Histogram<153> &p, float *out) const
	    {
		for (int i = 0; i < nr_dimensions_; ++i)
		out[i] = p.histogram[i];
	    }
	};
}

namespace pcl {
	template <>
	class DefaultPointRepresentation<Histogram<153> *> : public PointRepresentation<Histogram<153> *>
	{
	    public:

	    DefaultPointRepresentation()
	    {
		nr_dimensions_ = 153;
	    }

	    virtual void
	    copyToFloatArray (const Histogram<153> *p, float *out) const
	    {
		for (int i = 0; i < nr_dimensions_; ++i)
		out[i] = p->histogram[i];
	    }
	};
}

