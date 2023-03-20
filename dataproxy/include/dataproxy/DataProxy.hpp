#pragma once

#include <utils/SafeDeque.hpp>

namespace dataproxy
{

static constexpr bool UseBag = false;

using namespace utils;

template <typename T>
class DataProxy
{
public:
    using DataPtr = std::shared_ptr<concurrency::SafeDeque<T, UseBag>>;

protected:
    DataPtr mDataPtr;

public:
    explicit DataProxy(int size){
        mDataPtr = std::make_shared<concurrency::SafeDeque<T, UseBag>>(size);
    }
    
    const DataPtr get() const { return mDataPtr; }  
    
    virtual ~DataProxy() {};
};
    
} // namespace dataproxy





