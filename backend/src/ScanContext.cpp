#include <backend/ScanContext.hpp>

#include <geometry/trans.hpp>


namespace backend {

namespace context {

using namespace geometry;
using VContext = ScanContext::VContext;

ScanContext::ScanContext() {}

float ScanContext::xy2theta( const float & _x, const float & _y )
{
    if(_x >= 0 && _y >= 0) 
        return trans::rad2deg(atan(_y / _x));

    if(_x < 0 && _y >= 0) 
        return 180.0f - trans::rad2deg(atan(_y / (-_x)));

    if ( _x < 0 && _y < 0) 
        return 180 + trans::rad2deg((180/M_PI) * atan(_y / _x));

    if ( _x >= 0 && _y < 0)
        return 360 - trans::rad2deg((180/M_PI) * atan((-_y) / _x));
} // xy2theta

Context ScanContext::circshift(const Context &_mat, int _num_shift)
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        Context shifted_mat(_mat);
        return shifted_mat; // Early return 
    }

    Context shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols());
    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++)
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;
} // circshift

void ScanContext::addContext(const SourceType &input)
{
    Context sc = makeScanContext(input); // v1 
    VContext ringct = makeRingkeyFromScancontext( sc );
    VContext sectorct = makeSectorkeyFromScancontext( sc );

    polarcontexts_.push_back(sc);
    ringcontexts_.push_back(ringct);
    sectorcontexts_.push_back(sectorct); 
}

double ScanContext::computeSimularity(const Context& _sc1, const Context& _sc2)
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        Eigen::VectorXd col_sc1 = _sc1.col(col_idx);
        Eigen::VectorXd col_sc2 = _sc2.col(col_idx);
        
        if( col_sc1.norm() == 0 || col_sc2.norm() == 0 )
            continue; // don't count this sector pair. 

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }
    
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // distDirectSC

int ScanContext::fastAlignUsingVkey(const Context& _vkey1, const Context& _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = std::numeric_limits<double>::max();
    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
    {
        Context vkey2_shifted = circshift(_vkey2, shift_idx);

        Context vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} // fastAlignUsingVkey

std::pair<double, int> ScanContext::distanceBtnScanContext(const Context& _sc1, const Context& _sc2)
{
    // 1. fast align using variant key (not in original IROS18)
    Context vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    Context vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_sc_dist = std::numeric_limits<double>::max();
    for ( int num_shift: shift_idx_search_space )
    {
        Context sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = computeSimularity(_sc1, sc2_shifted);
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return std::make_pair(min_sc_dist, argmin_shift);
} // distanceBtnScanContext

Context ScanContext::makeScanContext(const pc_t& _scan_down)
{
    // TicToc t_making_desc;

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    Context desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    pt_t pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        // taking maximum z 
        if ( desc(ring_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
            desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;

    // t_making_desc.toc("PolarContext making");

    return desc;
} // SCManager::makeScancontext

VContext ScanContext::makeRingkeyFromScancontext(const Context& _desc)
{
    /* 
     * summary: rowwise mean vector
    */
    VContext invariant_key(_desc.rows());
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        VContext curr_row = _desc.row(row_idx);
        invariant_key(row_idx) = curr_row.mean();
    }

    return invariant_key;
} // SCManager::makeRingkeyFromScancontext


VContext ScanContext::makeSectorkeyFromScancontext(const Context& _desc)
{
    /* 
     * summary: columnwise mean vector
    */
    VContext variant_key(_desc.cols());
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++)
    {
        VContext curr_col = _desc.col(col_idx);
        variant_key(col_idx) = curr_col.mean();
    }

    return variant_key;
} // SCManager::makeSectorkeyFromScancontext

QueryResult ScanContext::query(int id)
{
    const VContext& key = ringcontexts_.at(id);
    const Context& desc = polarcontexts_.at(id);

    if(ringcontexts_.size() < NUM_EXCLUDE_RECENT + 1)   return {-1, 0};

    if(ringcontexts_.size() - ring_sub_.size() > NUM_EXCLUDE_RECENT + BUILD_TREE_GAP)
    {
        ring_sub_.clear();
        int n = ringcontexts_.size() - NUM_EXCLUDE_RECENT;
        ring_sub_.resize(n);
        for(int i=0; i<n; i++)  ring_sub_[i] = ringcontexts_[i];
        ring_kdtree_.setInput(&ring_sub_);
    }

    std::vector<size_t> k_indices;
    std::vector<double> k_sqr_distances;
    
    ring_kdtree_.nearestKSearch(key.data(), NUM_CANDIDATES_FROM_TREE, k_indices, k_sqr_distances);

    double min_dist = std::numeric_limits<double>::max();
    int nn_align = 0;
    int nn_idx = 0;

    for(int i=0; i<NUM_CANDIDATES_FROM_TREE; i++){
        const Context& cand_desc = polarcontexts_[k_indices[i]];
        auto res = distanceBtnScanContext(desc, cand_desc);

        double candidate_dist = res.first;
        int candidate_align = res.second;

        if(candidate_dist < min_dist)
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;
            nn_idx = k_indices[i];
        }
    }

    if(min_dist > SC_DIST_THRES)    return {-1, 0};
    
    return {nn_idx, trans::deg2rad(PC_UNIT_SECTORANGLE * nn_align)};
}

}

}