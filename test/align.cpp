#include <pcl/point_types.h>
#include <PCR/NdtRegister.hpp>

using namespace PCR;
using PointType = pcl::PointXYZI;


int main(int argc, char const *argv[])
{

    PointCloudRegister<PointType>::Ptr pcr = std::make_shared<NdtRegister<PointType>>();
    
    // pcr->scan2Map();
    
    return 0;
}
