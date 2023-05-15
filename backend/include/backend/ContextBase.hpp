#pragma once
#include <types/basic.hpp>

namespace backend {

namespace context {

using SourceType = pc_t;
using Context = Eigen::MatrixXd;
using QueryResult = std::pair<int, float>;

class ContextBase
{
public:

    virtual void addContext(const SourceType& input) = 0;
    virtual QueryResult query(int id) = 0;

    virtual size_t size() const = 0;
    // virtual double computeSimularity(size_t from, size_t to) = 0;
    virtual double computeSimularity(const Context& from, const Context& to) = 0;

    virtual void saveContext(){};
    virtual void loadContext(){};
    
    virtual ~ContextBase() = default;
};

}

}
