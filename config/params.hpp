#include <utils/NonCopyable.hpp>
#include <fstream>

#include <memory>
#include <mutex>

#include <nlohmann/json.hpp>

namespace config
{

using namespace utils;
using json = nlohmann::json;

class Params : public noncopyable::NonCopyable
{

private:

    using Ptr = std::shared_ptr<Params>;

    json _jps;
    mutable std::mutex _lk;
    
    Params(){
#ifdef CONFIG_FILE
        std::ifstream inf(CONFIG_FILE);
        // allow json use comments!!
        _jps = json::parse(inf, nullptr, true, true);
        inf.close();
#endif
    }

public:

    static json getInstance(){
        static auto p = std::shared_ptr<Params>(new Params());
        return p->get();
    }

    json get() const { 
        std::lock_guard<std::mutex> lock(_lk);
        return _jps; 
    }

};

}

