#pragma once

#include <backend/ContextBase.hpp>
#include <nanoflann/vov_adaptor.h>

namespace backend {

namespace context {

class DistContext final : public ContextBase
{
public:
    using Context = EigenTypes::V3<scalar_t>;
    using QueryResult = int;

    using Container = std::vector<Context>;
    using Kdtree = nanoflann::VectorOfVectorsKdTree<Container, scalar_t, 3>;

private:
    Container cont_;
    Kdtree kdtree_;

public:

    DistContext();

    virtual size_t size() const override { return cont_.size(); }

    virtual void addContext(const cbtype::Source& input) override;
    virtual cbtype::QueryResult query(int id) override;
    // virtual double computeSimularity(size_t from, size_t to) override;
    virtual double computeSimularity(const cbtype::Context& from, const cbtype::Context& to) override;

};

}

}