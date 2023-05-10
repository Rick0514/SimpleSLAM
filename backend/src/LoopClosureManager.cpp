#include <PCR/VgicpRegister.hpp>
#include <backend/ScanContext.hpp>

#include <backend/LoopClosureManager.hpp>

namespace backend {

LoopClosureManager::LoopClosureManager(const MapManagerPtr& mmp) : mmp_(mmp) 
{
    ctb_ = std::make_unique<context::ScanContext>();
    pcr_ = std::make_unique<PCR::VgicpRegister>();

    lc_thd_ = std::make_unique<trd::ResidentThread>(&LoopClosureManager::lcHandler, this);
}

void LoopClosureManager::lcHandler()
{
    
}

}