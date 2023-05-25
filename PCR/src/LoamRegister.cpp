#include <PCR/LoamRegister.hpp>

#include <geometry/manifolds.hpp>
#include <geometry/trans.hpp>
#include <time/tictoc.hpp>

#include <Eigen/Eigenvalues>

#include <nanoflann/pcl_adaptor.hpp>

namespace PCR
{
using namespace geometry;

struct LoamRegister::Kdtree
{
    nanoflann::PointCloudKdtree<pt_t, scalar_t> mKdtree;
    Kdtree() = default;
};

LoamRegister::LoamRegister() : mKdtree(std::make_unique<Kdtree>())
{
    // mKdtree.reset(new pcl::KdTreeFLANN<PointType>());
#ifdef DEBUG_DIR
    debug_file = std::ofstream(fmt::format("{}/{}.txt", DEBUG_DIR, "loam"));
#endif
}

template<unsigned int N>
bool LoamRegister::_extractPlaneCoeffs(const Eigen::Matrix<scalar_t, N, 3>& A, V3& x)
{
    Eigen::Matrix<scalar_t, N, 1> b;
    b.fill(-1);
    x = A.colPivHouseholderQr().solve(b);
    
    scalar_t x_norm = x.norm();

    for(int i=0; i<N; i++){
        if(std::abs(x.dot(A.row(i)) + 1.0) > mPlaneValidThresh * x_norm){
            DEBUG(debug_file, fmt::format("dist large: {}", hx.dot(A.row(i).homogeneous())));
            return false;
        }
    }
    return true;        
}

bool LoamRegister::_extractPlaneMatrix(const pt_t& pointInMap, const PC_cPtr& dst, Eigen::Matrix<scalar_t, mPlanePtsNum, 3>& A)
{
    std::vector<index_t> pointSearchInd;
    std::vector<scalar_t> pointSearchSqDis;

    // for pcl, because pcl point are all float
    scalar_t p[3];
    for(int i=0; i<3; i++)  p[i] = pointInMap.data[i];

    mKdtree->mKdtree.nearestKSearch(p, mPlanePtsNum, pointSearchInd, pointSearchSqDis);

    // DEBUG(debug_file, fmt::format("nks: {}", pointSearchSqDis[mPlanePtsNum-1]));
    if(pointSearchSqDis[mPlanePtsNum-1] < mKdtreeMaxSearchDist)
    {
        for(int j=0; j<mPlanePtsNum; j++){
            A(j, 0) = dst->points[pointSearchInd[j]].x;
            A(j, 1) = dst->points[pointSearchInd[j]].y;
            A(j, 2) = dst->points[pointSearchInd[j]].z;
        }

        // DEBUG(debug_file, A);
        return true;
    }

    return false;
}

void LoamRegister::_removeDegeneratePart(const M6& JtJ, V6& x){
    if(!degenerateProjSet){
        degenerateProjSet = true;

        V6 ev = JtJ.eigenvalues().real();
        
        std::stringstream ss;
        ss << "ev: " << ev.transpose();
        lg->debug("{}", ss.str());

        // DEBUG(debug_file, ss.str());

        M6 JtJ_copy = JtJ;
        for(int i=0; i<6; i++){
            if(ev(i) < mDegenerateThresh){
                isDegenerate = true;
                JtJ_copy.row(i).setZero();
            }
        }
        degenerateProj = JtJ.inverse() * JtJ;
    }
    
    // if(isDegenerate)    x = degenerateProj * x;
}

bool LoamRegister::scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res)
{
    isDegenerate = false;
    this->isConverge = false;
    degenerateProjSet = false;

    // construct J, E ==> J'*J = -J'E
    std::vector<Eigen::Matrix<scalar_t, 1, 6>> J_vec;
    std::vector<scalar_t> E_vec;
    
    common::time::tictoc tt;
    mKdtree->mKdtree.setInputCloud(dst);
    this->lg->debug("kdtree build idx cost: {}", tt);

    for(int it=0; it<iters; it++){

        J_vec.clear();
        E_vec.clear();

        // tt.tic();
        // scalar_t ta = 0;
        // scalar_t te = 0;

        #pragma omp parallel for num_threads(cores)
        for(int i=0; i<src->size(); i++){
            // trans point to map frame
            tt.tic();
            const auto& pointOri = src->points[i];
            pt_t pointInMap;
            V4 ori = pointOri.getVector4fMap().cast<scalar_t>();
            ori = res * ori;
            pointInMap.getVector4fMap() = ori.cast<float>();            
            // _pointAssociateToMap(&pointOri, &pointInMap, res);

            // ta += tt.elapsed().count();

            Eigen::Matrix<scalar_t, mPlanePtsNum, 3> A;

            // tt.tic();
            if(_extractPlaneMatrix(pointInMap, dst, A))
            {
                // extract plane
                V3 x;
                if(_extractPlaneCoeffs<mPlanePtsNum>(A, x)){
                    // check good point to optimize
                    V3 p = pointInMap.getVector3fMap().cast<scalar_t>();
                    scalar_t dist = _dist(x, p);

                    scalar_t s = 1 - 0.9 * fabs(dist) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));
                    
                    // DEBUG(debug_file, fmt::format("weight: {}", s));
                    if (s > mPointValidThresh) {
                        // good point is selected!!
                        #pragma omp critical
                        {
                            E_vec.emplace_back(s * _dist(x, p));                            
                            Eigen::Matrix<scalar_t, 3, 6> jse3;
                            manifolds::J_SE3(p, jse3);
                            J_vec.emplace_back(s * _J_e_wrt_x(x).transpose() * jse3);
                        }
                    }
                }
            }
            // te += tt.elapsed().count();
        }

        // this->lg->debug("build J cost: {}", tt);
        // this->lg->debug("ta cost: {}", ta);
        // this->lg->debug("te cost: {}", te);

        // make J and E
        auto n = J_vec.size();
        this->lg->debug("optimize size: {}", n);
        if(n < 6){
            this->lg->error("not enough valid point({}) to optimize!", n);
            break;
        }

        tt.tic();
        using MatrixX6 = Eigen::Matrix<scalar_t, Eigen::Dynamic, 6>;
        using VectorX = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
        MatrixX6 J(n, 6);
        VectorX E(n, 1);
        for(int i=0; i<n; i++){
            J.row(i) = J_vec[i];
            E(i) = E_vec[i];
        }
        M6 JtJ = J.transpose() * J;
        V6 JtE = J.transpose() * E;

        // ColMajor 6xn --> vector<1x6>
        // using MatrixX6 = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;    
        // using VectorX = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
        // Eigen::Map<MatrixX6> J(J_vec.data()->data(), 6, n);
        // Eigen::Map<VectorX> E(E_vec.data(), n, 1);
        // M6 JtJ = J * J.transpose();
        // V6 JtE = J * E;

        V6 x = JtJ.ldlt().solve(-JtE);
        this->lg->debug("solve x cost: {}", tt);

        // check converge
        if(x.head<3>().norm() <= mPosConverge && x.tail<3>().norm() <= mRotConverge){
            this->isConverge = true;
            // this->lg->info("converge at iter {}!", it);
            break;
        }
        
        // std::stringstream ss;
        // ss << "optimize-" << it << " ==> res: " << x.transpose();
        // DEBUG(debug_file, ss.str());

        // _removeDegeneratePart(JtJ, x);
        M4 Tse3;
        manifolds::exp(x, Tse3);
        
        res.matrix() = Tse3 * res.matrix();
    }

    // make it standard SE3 !!!
    trans::T2SE3(res.matrix());

    return this->isConverge;
}

LoamRegister::~LoamRegister() = default;

} // namespace PCR
