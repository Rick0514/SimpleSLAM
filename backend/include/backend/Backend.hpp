#include <types/PCLTypes.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace backend
{

using namespace PCLTypes;

class Backend
{
private:

    PCxyz::Ptr mSubMap;
    pcl::KdTreeFLANN<Pxyz>::Ptr mSubMapKdtree;

public:
    Backend(/* args */);

    const pcl::KdTreeFLANN<Pxyz>::Ptr& getSubMapKdtree() const;
    const PCxyz::Ptr& getSubMap() const;
    
    ~Backend();
};
    
} // namespace backend



