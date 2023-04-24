#include <config/params.hpp>
#include <utils/Logger.hpp>

using namespace utils;
using namespace config;

int main()
{
    auto lg = logger::Logger::getInstance();

    auto cfg = Params::getInstance();

    lg->info("cfg: {}", cfg.dump());
    
    return 0;
}