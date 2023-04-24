#include <utils/NonCopyable.hpp>
#include <utils/Logger.hpp>

#include <memory>

#include <nlomann/json.hpp>

namespace config
{

using namespace utils;
using json = nlohmann::json;

class Params : public noncopyable::NonCopyable
{

private:

    using Ptr = std::shared_ptr<Params>;

    json _jps;
    logger::Logger::Ptr lg;
    
    Params(){
    #ifdef CONFIG
        std::ifstream inf(CONFIG);
        inf >> _jps;
        inf.close();
    #endif

        lg->info("get config: {}", _jps.dump());
    }

public:

    static Ptr getInstance(){
        static auto p = std::make_shared<Params>();
        return p;
    }

}

}

