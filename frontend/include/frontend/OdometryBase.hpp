#include <utils/SafeDeque.hpp>
#include <types/EigenTypes.hpp>

#include <memory>

namespace frontend
{

static constexpr bool UseBag = false;

using namespace EigenTypes;
using namespace utils;

class OdometryBase
{
protected:

public:
    OdometryBase();

    virtual void generateOdom() = 0;

    ~OdometryBase();
};
    
} // namespace frontend

