#pragma once
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace utils {

namespace thread {

class ResidentThread
{
private:

    std::atomic_bool running{true};
    std::atomic_bool stop{false};
    std::thread thd;

    mutable std::mutex lk;
    std::condition_variable cv;

public:

    template<typename Func, typename... Args>    
    explicit ResidentThread(Func&& f, Args&&... args)
    {
        auto fb = std::bind(std::forward<Func>(f), std::forward<Args>(args)...);
        thd = std::move(std::thread([&](){
            while(running.load()){
                std::unique_lock<std::mutex> lock(lk);
                cv.wait(lk, [&](){ return !stop; });
                fb();
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

    void Close(){
        cv.notify_all();
        running.store(true);
    }
    
    ~ResidentThread()
    {
        if(thd.joinable())   thd.join();
    }

}

}
}