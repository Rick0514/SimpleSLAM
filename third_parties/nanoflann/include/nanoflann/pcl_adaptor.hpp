#pragma once
#include "nanoflann.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace nanoflann {

using namespace pcl;

template <typename PointType, typename Scalar>
class PointCloudKdtree
{

protected:
    static constexpr int Dim = 3;

    struct PointCloudAdaptor
    {
        inline size_t kdtree_get_point_count() const{
            if(_pc) return _pc->points.size();
            return 0;
        }
        inline Scalar kdtree_get_pt(const size_t idx, int dim) const
        {
            const PointType& p = _pc->points[idx];
            return p.data[dim];
        }
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
        typename PointCloud<PointType>::ConstPtr _pc;
    }_adaptor;

    using metric_t = typename metric_L2_Simple::traits<Scalar, PointCloudAdaptor>::distance_t;
    using kdtree_t = KDTreeSingleIndexAdaptor<metric_t, PointCloudAdaptor, Dim, size_t>;

    kdtree_t _kdtree;

public:
    PointCloudKdtree() : _kdtree(Dim, _adaptor){}

    void setInputCloud(const typename PointCloud<PointType>::ConstPtr& cloud)
    {
        _adaptor._pc = cloud;
        _kdtree.buildIndex();
    }

    size_t nearestKSearch(const PointType &point, int k,
            std::vector<size_t> &k_indices,
            std::vector<Scalar> &k_sqr_distances) const
    {
        k_indices.resize(k);
        k_sqr_distances.resize(k);
        
        KNNResultSet<Scalar> resultSet(k);
        resultSet.init(k_indices.data(), k_sqr_distances.data());
        _kdtree.findNeighbors(resultSet, point.data);
        return resultSet.size();
    }

    size_t radiusSearch(const PointType &point, Scalar radius, std::vector<size_t> &k_indices,
        std::vector<Scalar> &k_sqr_distances, bool sorted=false) const
    {
        std::vector<ResultItem<size_t, Scalar>> indices_dist;
        SearchParameters sp(0, sorted);
        _kdtree.radiusSearch(point.data, radius, indices_dist, sp);

        auto n = indices_dist.size();
  
        k_indices.resize(n);
        k_sqr_distances.resize(n);
        for (int i = 0; i < n; i++) {
            k_indices[i] = indices_dist[i].first;
            k_sqr_distances[i] = indices_dist[i].second;
        }
        return n;
    }

};

}