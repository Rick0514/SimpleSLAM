/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-16 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#pragma once

#include "nanoflann.hpp"

#include <vector>

// ===== This example shows how to use nanoflann with these types of containers: =======
//typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;   // This requires #include <Eigen/Dense>
// =====================================================================================


/** A simple vector-of-vectors adaptor for nanoflann, without duplicating the storage.
  *  The i'th vector represents a point in the state space.
  *
  *  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality for the points in the data set, allowing more compiler optimizations.
  *  \tparam num_t The type of the point coordinates (typically, double or float).
  *  \tparam Distance The distance metric to use: nanoflann::metric_L1, nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
  *  \tparam IndexType The type for indices in the KD-tree index (typically, size_t of int)
  */
namespace nanoflann {

template <class VectorOfVectorsType, typename num_t, int DIM, class Distance = nanoflann::metric_L2, typename IndexType = size_t>
class VectorOfVectorsKdTree
{
protected:

    struct VectorOfVectorsAdaptor
    {
        VectorOfVectorsAdaptor() = default;
        // Must return the number of data points
        inline IndexType kdtree_get_point_count() const { return m_data->size(); }
        // Returns the dim'th component of the idx'th point in the class:
        inline num_t kdtree_get_pt(const IndexType idx, int dim) const { return (*m_data)[idx](dim); }
    	template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }

        VectorOfVectorsType* m_data;
    }_adaptor;

    typedef VectorOfVectorsAdaptor self_t;
	typedef typename Distance::template traits<num_t, self_t>::distance_t metric_t;
	typedef KDTreeSingleIndexAdaptor<metric_t, self_t, DIM, IndexType> kdtree_t;

    kdtree_t _kdtree;

public:

    VectorOfVectorsKdTree() : _kdtree(DIM, _adaptor) {}

    void setInput(VectorOfVectorsType* data){
        _adaptor.m_data = data;
        _kdtree.buildIndex();
    }

    size_t nearestKSearch(const num_t* const point, int k, std::vector<size_t> &k_indices,
        std::vector<num_t> &k_sqr_distances) const
    {
        return _kdtree.knnSearch(point, k, k_indices.data(), k_sqr_distances.data());
    }

}; // end of KDTreeVectorOfVectorsAdaptor

}

