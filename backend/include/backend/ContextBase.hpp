#pragma once
#include <types/basic.hpp>
#include <utils/Logger.hpp>
#include <any>

#define UsingClassType(type, classname)     \
using type = classname::type;               \

namespace backend {

namespace context {

namespace cbtype {
    using Source = std::any;
    using Context = std::any;
    using QueryResult = std::any;
}

class ContextBase
{

protected:
    std::shared_ptr<utils::logger::Logger> lg;
    ContextBase() { lg = utils::logger::Logger::getInstance(); }

public:

    virtual void addContext(const cbtype::Source& input) = 0;
    virtual cbtype::QueryResult query(int id) = 0;

    virtual size_t size() const = 0;
    // virtual double computeSimularity(size_t from, size_t to) = 0;
    virtual double computeSimularity(const cbtype::Context& from, const cbtype::Context& to) = 0;

    virtual void saveContext(){};
    virtual void loadContext(){};
    
    virtual ~ContextBase() = default;
};

}

}
