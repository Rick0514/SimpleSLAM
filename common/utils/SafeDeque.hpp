#include <deque>
#include <mutex>
#include <condition_variable>

namespace utils
{
namespace concurrency
{
    
template<typename T, bool blocking>
class SafeDeque
{
private:

    int mSize;
    std::deque<T> mDq;

    std::mutex mLock;
    std::condition_variable mCv;
    
public:
    explicit SafeDeque(int size) : mSize(size) {};
    
    SafeDeque(const SafeDeque&) = delete;
    SafeDeque(SafeDeque&&) = delete;
    SafeDeque& operator= (const SafeDeque&) = delete;
    SafeDeque& operator= (SafeDeque&&) = delete;

    bool empty() const noexcept;
    int size() const noexcept;

    T at(int idx) const;

    void push_back(const T&);

    void pop_back();
    void pop_front();

    bool front(T&) const;
    bool back(T&) const;

    ~SafeDeque(){};
};


template <typename T, bool blocking>
bool SafeDeque<T, blocking>::empty() const noexcept
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.empty();
}

template <typename T, bool blocking>
int SafeDeque<T, blocking>::size() const noexcept
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.size();
}

template <typename T, bool blocking>
T SafeDeque<T, blocking>::at(int idx) const
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.at(idx);
}

template <typename T, bool blocking>
void SafeDeque<T, blocking>::push_back(const T& item)
{
    std::unique_lock<std::mutex> lk(mLock);
    if constexpr (blocking){
        mCv.wait(lk, [&](){return mDq.size() < mSize;});
    }else{        
        if(mDq.size() == mSize){
            mDq.pop_front();
        }        
    }

    mDq.emplace_back(item);
}

template <typename T, bool blocking>
void SafeDeque<T, blocking>::pop_back()
{
    std::lock_guard<std::mutex> lk(mLock);
    if(!mDq.empty())    mDq.pop_back();
    mCv.notify_one();
}

template <typename T, bool blocking>
void SafeDeque<T, blocking>::pop_front()
{
    std::lock_guard<std::mutex> lk(mLock);
    if(!mDq.empty())    mDq.pop_front();
    mCv.notify_one();
}

template <typename T, bool blocking>
bool SafeDeque<T, blocking>::front(T& f) const
{
    std::lock_guard<std::mutex> lk(mLock);
    if(mDq.empty()) return false;
    f = mDq.front();
    return true;
}

template <typename T, bool blocking>
bool SafeDeque<T, blocking>::back(T& b) const
{
    std::lock_guard<std::mutex> lk(mLock);
    if(mDq.empty()) return false;
    b = mDq.back();
    return true;
}

} // namespace concurrency

} // namespace utils



