#pragma once

#include <backend/ContextBase.hpp>
#include <nanoflann/vov_adaptor.h>

namespace backend {

namespace context {

class DistContext final : public ContextBase
{
private:
    
    using context_t = Eigen::Matrix<scalar_t, 2, 1>;

    std::vector<context_t> cont_;

public:

    DistContext();

    virtual size_t size() const override { return cont_.size(); }

    virtual void addContext(const SourceType& input) override;
    virtual QueryResult query(int id) override;
    // virtual double computeSimularity(size_t from, size_t to) override;
    virtual double computeSimularity(const Context& from, const Context& to) override;

};

}

}