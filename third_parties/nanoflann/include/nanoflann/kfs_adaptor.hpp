#pragma once
#include "nanoflann.hpp"

namespace nanoflann
{

static constexpr int DIM = 3;

template<typename Container, typename Scalar=double, int Dim=DIM, 
    typename Distance=metric_L2, typename IndexType=size_t>
struct KeyFramesAdaptor
{
    using self_t = KeyFramesAdaptor<Container, Scalar, Dim>;
    using metric_t = typename Distance::template traits<Scalar, self_t>::distance_t;
    using index_t = KDTreeSingleIndexAdaptor<metric_t, self_t, Dim, IndexType>;

    index_t* index = nullptr;
    const Container& m_data;

    KeyFramesAdaptor(const Container& mat, const int leaf_max_size = 10,
        const unsigned int n_thread_build = 1) : m_data(mat)
    {
        index = new index_t(DIM, *this,
                KDTreeSingleIndexAdaptorParams(
                    leaf_max_size,
                    KDTreeSingleIndexAdaptorFlags::None,
                    n_thread_build
                )
            );
    }

    ~KeyFramesAdaptor() { delete index; }

    void query(const Scalar* query_point, const size_t num_closest,
        IndexType* out_indices, Scalar* out_distances_sq) const
    {
        KNNResultSet<Scalar, IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances_sq);
        index->findNeighbors(resultSet, query_point);
    }

    const self_t& derived() const { return *this; }
    self_t&       derived() { return *this; }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return m_data.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline Scalar kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        return m_data[idx].translation(dim);
    }

};

}