#include <PCR/NdtRegister.hpp>
#include <pclomp/ndt_omp.h>

namespace PCR
{
struct NdtRegister::Ndt
{
    using ndt_t = pclomp::NormalDistributionsTransform<pt_t, pt_t>;
    ndt_t _ndt_omp;
    
    Ndt(){
        _ndt_omp.setResolution(resol);
        _ndt_omp.setNumThreads(constant::numCores);
        _ndt_omp.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    }
};

NdtRegister::NdtRegister() : _ndt(std::make_unique<Ndt>()) {}

bool NdtRegister::scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res)
{
    _ndt->_ndt_omp.setInputTarget(dst);
    _ndt->_ndt_omp.setInputSource(src);

    pc_t aligned;
    _ndt->_ndt_omp.align(aligned, res.matrix().cast<float>());
    res.matrix() = _ndt->_ndt_omp.getFinalTransformation().cast<scalar_t>();

    return _ndt->_ndt_omp.hasConverged();
}

NdtRegister::~NdtRegister() = default;

} // namespace PCR
