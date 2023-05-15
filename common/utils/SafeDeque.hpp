#pragma once
#include <deque>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace utils
{
namespace concurrency
{

// read-only buffer!!
// shared ptr buffer!!
template<typename T>
class SafeDeque
{
private:
    using Tsptr = std::shared_ptr<T>;

    int mSize;
    std::deque<Tsptr> mDq;

    mutable std::mutex mLock;
    std::condition_variable mCv;

    std::atomic_bool running{true};
    
public:
    explicit SafeDeque(int size) : mSize(size) {};

    // uncopyable
    SafeDeque(const SafeDeque&) = delete;
    SafeDeque& operator= (const SafeDeque&) = delete;

    void resize(int sz) noexcept;
    bool empty() const noexcept;
    int size() const noexcept;

    const Tsptr at(int idx) const;

    template <bool blocking>
    void push_back(const Tsptr&);

    template <bool blocking>
    void push_back(Tsptr&&);

    void pop_front();
    void pop_back();

    const Tsptr front() const;
    const Tsptr back() const;

    Tsptr consume_front();
    Tsptr consume_back();

    void clear() noexcept;

    void abort() noexcept;

    // provide a more fine-grain and thread-safe api to operate deque
    std::mutex& getLock() noexcept;
    std::condition_variable& getCv() noexcept;
    std::deque<Tsptr>& getDequeInThreadUnsafeWay() noexcept;

    ~SafeDeque() = default;
};

template <typename T>
void SafeDeque<T>::resize(int sz) noexcept
{
    std::lock_guard<std::mutex> lk(mLock);
    mSize = sz;
}

template <typename T>
bool SafeDeque<T>::empty() const noexcept
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.empty();
}

template <typename T>
int SafeDeque<T>::size() const noexcept
{
    std::lock_guard<std::mutex> lk(mLock);
    return (int)mDq.size();
}

template <typename T>
const std::shared_ptr<T> SafeDeque<T>::at(int idx) const
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.at(idx);
}

template <typename T>
template <bool blocking>
void SafeDeque<T>::push_back(const std::shared_ptr<T>& item)
{
    std::unique_lock<std::mutex> lk(mLock);
    
    if constexpr (blocking){
        mCv.wait(lk, [&](){return (running.load() ? mDq.size() < mSize : true);});
    }else{        
        if(mDq.size() == mSize){
            mDq.pop_front();
        }        
    }
    if(running.load())  mDq.emplace_back(item);
}

template <typename T>
template <bool blocking>
void SafeDeque<T>::push_back(std::shared_ptr<T>&& item)
{
    std::unique_lock<std::mutex> lk(mLock);
    
    if constexpr (blocking){
        mCv.wait(lk, [&](){return (running.load() ? mDq.size() < mSize : true);});
    }else{        
        if(mDq.size() == mSize){
            mDq.pop_front();
        }        
    }

    if(running.load())  mDq.emplace_back(std::move(item));
}

template <typename T>
void SafeDeque<T>::pop_back()
{
    std::lock_guard<std::mutex> lk(mLock);
    if(!mDq.empty())    mDq.pop_back();
    mCv.notify_all();
}

template <typename T>
void SafeDeque<T>::pop_front()
{
    std::lock_guard<std::mutex> lk(mLock);
    if(!mDq.empty())    mDq.pop_front();
    mCv.notify_all();
}

template <typename T>
const std::shared_ptr<T> SafeDeque<T>::front() const
{
    std::lock_guard<std::mutex> lk(mLock);
    if(mDq.empty()) return std::shared_ptr<T>(nullptr);
    return mDq.front();
}

template <typename T>
const std::shared_ptr<T> SafeDeque<T>::back() const
{
    std::lock_guard<std::mutex> lk(mLock);
    if(mDq.empty()) return std::shared_ptr<T>(nullptr);
    return mDq.back();
}

template <typename T>
std::shared_ptr<T> SafeDeque<T>::consume_front()
{
    std::lock_guard<std::mutex> lk(mLock);
    if(mDq.empty()){
        return std::shared_ptr<T>(nullptr);
    }
    auto ptr = mDq.front();
    mDq.pop_front();
    mCv.notify_all();
    return ptr;
}

template <typename T>
std::shared_ptr<T> SafeDeque<T>::consume_back()
{
    std::lock_guard<std::mutex> lk(mLock);
    if(mDq.empty()) return std::shared_ptr<T>(nullptr);
    auto ptr = mDq.back();
    mDq.pop_back();
    mCv.notify_all();
    return ptr;
}

template <typename T>
void SafeDeque<T>::abort() noexcept
{
    running.store(false);
    mCv.notify_all();
}

template <typename T>
void SafeDeque<T>::clear() noexcept
{
    std::lock_guard<std::mutex> lk(mLock);
    mDq.clear();
    mCv.notify_all();
}


template <typename T>
std::mutex& SafeDeque<T>::getLock() noexcept
{
    return mLock;
}

template <typename T>
std::condition_variable& SafeDeque<T>::getCv() noexcept
{
    return mCv;
}

template<typename T>
std::deque<std::shared_ptr<T>>& SafeDeque<T>::getDequeInThreadUnsafeWay() noexcept
{
    return mDq;
}



} // namespace concurrency

} // namespace utils



