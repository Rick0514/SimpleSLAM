#pragma once
#include <mutex>

namespace utils {
namespace trd {

template<typename T>
class AtomicVar
{
private:
    mutable std::mutex lock;    
    T data;

public:

    AtomicVar() = default;
    AtomicVar(const T& a) : data(a) {}

    T load() const
    {
        std::lock_guard<std::mutex> lk(lock);
        return data;
    }

    void store(const T& a)
    {
        std::lock_guard<std::mutex> lk(lock);
        data = a;
    }
};

}
}
