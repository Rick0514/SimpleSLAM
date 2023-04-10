#pragma once
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <utils/NonCopyable.hpp>

namespace utils {

namespace trd {

class ResidentThread : public noncopyable::NonCopyable
{
private:

    std::atomic_bool running;
    std::atomic_bool stop;
    std::thread thd;

    mutable std::mutex lk;
    std::condition_variable cv;
    std::function<void()> func;

public:

    ResidentThread() noexcept = default;

    template<typename Func, typename... Args>    
    explicit ResidentThread(Func&& f, Args&&... args) : running(true), stop(false)
    {
        func = std::bind(std::forward<Func>(f), std::forward<Args>(args)...);
        thd = std::move(std::thread([&](){
            while(running.load()){
                std::unique_lock<std::mutex> lock(lk);
                cv.wait(lock, [&](){ return !stop; });
                func();
            }
        }));
    }

    void Stop() {
        stop.store(true);
    }

    void Resume() {
        stop.store(false);
        cv.notify_all();
    }

    bool IsActive() {
        return thd.get_id() != std::thread::id();
    }

    ~ResidentThread()
    {
        running.store(false);
        Resume();
        if(thd.joinable())   thd.join();
    }

};

}
}