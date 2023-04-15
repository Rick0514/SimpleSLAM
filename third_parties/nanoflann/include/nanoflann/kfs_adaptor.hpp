#pragma once
#include "nanoflann.hpp"

// for vector which contain eigen vector data

namespace nanoflann
{

template<typename Container, typename Scalar, int Dim,
    typename Distance=metric_L2_Simple, typename IndexType=size_t>
class KeyFramesKdtree
{

protected:
    using elem_t = typename Container::value_type;
    
    struct KeyFramesAdaptor
    {
        KeyFramesAdaptor(const Container& c) : _data(c){}
        inline IndexType kdtree_get_point_count() const { return _data.size(); }
        inline Scalar kdtree_get_pt(const IndexType idx, int dim) const {
            return _data[idx].pose.translation()(dim);
        }
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
        const Container& _data;
    }_adaptor;

    using metric_t = typename Distance::template traits<Scalar, KeyFramesAdaptor>::distance_t;
    using kdtree_t = KDTreeSingleIndexAdaptor<metric_t, KeyFramesAdaptor, Dim, IndexType>;

    kdtree_t _kdtree;

public:

    KeyFramesKdtree() = delete;
    KeyFramesKdtree(const Container& c) : _adaptor(c), _kdtree(Dim, _adaptor){}

    void setInput(const Container& mat){
        _adaptor._data = mat;
        _kdtree.buildIndex();
    }

    template <typename EigenPT>
    IndexType nearestKSearch(const EigenPT &point, IndexType k, std::vector<IndexType> &k_indices,
        std::vector<Scalar> &k_sqr_distances) const
    {
        k_indices.resize(k);
        k_sqr_distances.resize(k);
        
        KNNResultSet<Scalar> resultSet(k);
        resultSet.init(k_indices.data(), k_sqr_distances.data());
        _kdtree.findNeighbors(resultSet, point.data());

        return resultSet.size();
    }
    
    template <typename EigenPT>
    IndexType radiusSearch (const EigenPT &point, Scalar radius, std::vector<IndexType> &k_indices,
        std::vector<Scalar> &k_sqr_distances, bool sorted=false) const
    {
        std::vector<ResultItem<size_t, Scalar>> indices_dists;
        SearchParameters sp(0, sorted);
        _kdtree.radiusSearch(point.data(), radius, indices_dists, sp);

        auto n = indices_dists.size();
        k_indices.resize(n);
        k_sqr_distances.resize(n);
        for (int i = 0; i < n; i++) {
            k_indices[i] = indices_dists[i].first;
            k_sqr_distances[i] = indices_dists[i].second;
        }

        return n;
    }
};


}

