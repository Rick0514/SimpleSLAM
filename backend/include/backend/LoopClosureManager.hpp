#pragma once

#include <utils/Thread.hpp>

namespace PCR { class PointCloudRegister; }
namespace frontend { class MapManager; }

namespace backend {

using namespace utils;
namespace context { class ContextBase; }


class LoopClosureManager
{

protected:

    using MapManagerPtr = std::shared_ptr<frontend::MapManager>;

    std::unique_ptr<context::ContextBase> ctb_;
    std::unique_ptr<PCR::PointCloudRegister> pcr_;
    MapManagerPtr mmp_;

    std::unique_ptr<trd::ResidentThread> lc_thd_;

public:
    
    LoopClosureManager(const MapManagerPtr& mmp);

    void lcHandler();

};

}
