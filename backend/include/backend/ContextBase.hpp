#pragma once
#include <vector>

namespace backend {

template <typename SourceType, typename ContextType>
class ContextBase
{
protected:
    std::vector<ContextType> mContextStorage; 

public:

    virtual ContextType makeContext(const SourceType& input) = 0;
    virtual double computeSimularity(size_t from, size_t to) = 0;
    virtual double computeSimularity(const ContextType& from, const ContextType& to) = 0;

    virtual void saveContext(){};
    virtual void loadContext(){};
    
};

}
