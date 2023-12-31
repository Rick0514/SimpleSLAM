#pragma once

#include <memory>
#include <types/EigenTypes.hpp>
#include <utils/SafeDeque.hpp>
#include <utils/Logger.hpp>

using namespace EigenTypes;
using namespace utils;

namespace dataproxy
{

template <typename T>
class DataProxy
{
public:

    using SafeDequeType = concurrency::SafeDeque<T>;
    using DataPtr = std::shared_ptr<SafeDequeType>;

protected:
    DataPtr mDataPtr;
    std::shared_ptr<utils::logger::Logger> mLg;

public:
    explicit DataProxy(int size){
        mDataPtr = std::make_shared<SafeDequeType>(size);
        mLg = utils::logger::Logger::getInstance();
    }
    
    DataPtr get() const { return mDataPtr; }  
    
    virtual ~DataProxy() { mDataPtr->abort(); };
};
    
} // namespace dataproxy





