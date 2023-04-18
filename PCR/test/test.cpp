#include <types/basic.hpp>
#include <time/tictoc.hpp>
#include <utils/Logger.hpp>

using namespace EigenTypes;
using namespace PCLTypes;
using namespace utils;

template <typename PointType>
void _pointAssociateToMap(PointType const * const pi, PointType * const po, const pose_t& p)
{
    const M3<scalar_t>& rot = p.rotation();
    const V3<scalar_t>& t = p.translation();
    po->x = rot(0,0) * pi->x + rot(0,1) * pi->y + rot(0,2) * pi->z + t(0);
    po->y = rot(1,0) * pi->x + rot(1,1) * pi->y + rot(1,2) * pi->z + t(1);
    po->z = rot(2,0) * pi->x + rot(2,1) * pi->y + rot(2,2) * pi->z + t(2);
}

int main()
{
    auto lg = logger::Logger::getInstance();

    pose_t p;
    p.setIdentity();
    
    pc_t::Ptr src(new pc_t);
    pt_t pxyz(1, 1, 1);
    src->points.push_back(pxyz);

    common::time::tictoc tt;

    for(int i=0; i<1e5; i++){
        // below 3 lines in LoamRegister behave strange in clang++-12 in ubuntu18.04
        // 10times++ slower than using GNU compiler, but why?? cannot reappear it!!
        const pt_t& pr = src->points[0];
        pt_t tmp;
        _pointAssociateToMap(&pr, &tmp, p);
    }
    
    // seems normal no matter with clang or gnu, but appear in LoamRegister
    lg->info("elapsed : {}ms", tt.elapsed().count() * 1000);

    return 0;
}