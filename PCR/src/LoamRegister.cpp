#include <PCR/LoamRegister.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry/manifolds.hpp>
#include <Eigen/Eigenvalues>

namespace PCR
{

template<typename PointType>
LoamRegister<PointType>::LoamRegister(){
    mKdtree.reset(new pcl::KdTreeFLANN<PointType>());

#ifdef DEBUG_DIR
    debug_file = std::ofstream(fmt::format("{}/{}.txt", DEBUG_DIR, "loam"));
#endif
}

template<typename PointType>
template<unsigned int N>
bool LoamRegister<PointType>::_extractPlaneCoeffs(const Eigen::Matrix<double, N, 3>& A, V4d& hx)
{
    Eigen::Matrix<double, N, 1> b;
    V3d x;
    b.fill(-1);
    x = A.colPivHouseholderQr().solve(b);
    hx = x.homogeneous();
    
    for(int i=0; i<N; i++){
        if(std::abs(hx.dot(A.row(i).homogeneous())) > mPlaneValidThresh){
            DEBUG(debug_file, fmt::format("dist large: {}", hx.dot(A.row(i).homogeneous())));
            return false;
        }
    }
    return true;        
}

template<typename PointType>
bool LoamRegister<PointType>::_extractPlaneMatrix(const PointType& pointInMap, PC_cPtr& dst, Eigen::Matrix<double, mPlanePtsNum, 3>& A)
{
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    mKdtree->nearestKSearch(pointInMap, mPlanePtsNum, pointSearchInd, pointSearchSqDis);

    DEBUG(debug_file, fmt::format("nks: {}", pointSearchSqDis[mPlanePtsNum-1]));
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

template<typename PointType>
void LoamRegister<PointType>::_removeDegeneratePart(const M6d& JtJ, V6d& x){
    if(!degenerateProjSet){
        degenerateProjSet = true;

        V6d ev = JtJ.eigenvalues().real();
        M6d JtJ_copy = JtJ;
        for(int i=0; i<6; i++){
            if(ev(i) < mDegenerateThresh){
                isDegenerate = true;
                JtJ_copy.row(i).setZero();
            }
        }
        degenerateProj = JtJ.inverse() * JtJ;
    }
    
    if(isDegenerate)    x = degenerateProj * x;
}

template<typename PointType>
bool LoamRegister<PointType>::scan2Map(PC_cPtr& src, PC_cPtr& dst, Pose6d& res)
{
    isDegenerate = false;
    this->isConverge = false;
    degenerateProjSet = false;

    // construct J, E ==> J'*J = -J'E
    std::vector<Eigen::Matrix<double, 1, 6>> J_vec;
    std::vector<double> E_vec;
    
    mKdtree->setInputCloud(dst);

    for(int it=0; it<iters; it++){

        J_vec.clear();
        E_vec.clear();

        for(int i=0; i<src->points.size(); i++){
            // trans point to map frame
            const PointType& pointOri = src->points[i];
            PointType pointInMap;
            _pointAssociateToMap(&pointOri, &pointInMap, res);

            Eigen::Matrix<double, mPlanePtsNum, 3> A;
            if(_extractPlaneMatrix(pointInMap, dst, A))
            {
                // extract plane
                V4d hx;
                if(_extractPlaneCoeffs<mPlanePtsNum>(A, hx)){
                    // check good point to optimize
                    V3d p(pointInMap.x, pointInMap.y, pointInMap.z);
                    float dist = hx.dot(p.homogeneous());

                    float s = 1 - 0.9 * fabs(dist) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));
                    
                    DEBUG(debug_file, fmt::format("weight: {}", s));
                    if (s > mPointValidThresh) {
                        // good point is selected!!
                        E_vec.emplace_back(_error(hx, p));
                        Eigen::Matrix<double, 3, 6> jse3;
                        manifolds::J_SE3(p, jse3);
                        J_vec.emplace_back(_J_e_wrt_x(hx).transpose() * jse3);
                    }
                }
            }
        }

        // make J and E
        int n = J_vec.size();
        spdlog::debug("optimize size: {}", n);
        if(n < 6){
            spdlog::error("not enough valid point({}) to optimize!", n);
            break;
        }

        Eigen::MatrixXd J(n, 6);
        Eigen::MatrixXd E(n, 1);

        for(int i=0; i<n; i++){
            J.row(i) = J_vec[i];
            E(i) = E_vec[i];
        }

        M6d JtJ = J.transpose() * J;
        V6d JtE = J.transpose() * E;
        
        V6d x = JtJ.ldlt().solve(-JtE);
        
        // check converge
        if(x.head<3>().norm() <= mPosConverge && x.tail<3>().norm() <= mRotConverge){
            this->isConverge = true;
            spdlog::debug("converge at iter {}!", it);
            break;
        }
        
        std::stringstream ss;
        ss << "optimize res: " << x.transpose();
        DEBUG(debug_file, ss.str());

        // _removeDegeneratePart(JtJ, x);
        M4d Tse3;
        manifolds::exp(x, Tse3);
        
        res.matrix() = Tse3 * res.matrix();
    }

    return this->isConverge;
}

PCRTemplateInstantiateExplicitly(LoamRegister)

} // namespace PCR

