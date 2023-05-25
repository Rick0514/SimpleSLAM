#include <backend/DistContext.hpp>

namespace backend {
namespace context {

UsingClassType(Context, DistContext)
UsingClassType(QueryResult, DistContext)

DistContext::DistContext()
{

}

void DistContext::addContext(const cbtype::Source &input)
{
    auto ct = std::any_cast<const Context&>(input);
    cont_.emplace_back(ct);    
}

double DistContext::computeSimularity(const cbtype::Context &from, const cbtype::Context &to)
{
    return 0.0;
}

cbtype::QueryResult DistContext::query(int id)
{
    QueryResult ans = -1;


    return ans;
}

}
}