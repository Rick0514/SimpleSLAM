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

    T at(int idx) const;

    void push_back(T&&);

    void pop_back();
    void pop_front();

    T front() const;
    T back() const;

    ~SafeDeque();
};


template <typename T, bool blocking>
bool SafeDeque<T, blocking>::empty() const noexcept
{
    return mDq.empty();
}

template <typename T, bool blocking>
T SafeDeque<T, blocking>::at(int idx) const
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.at(idx);
}

template <typename T, bool blocking>
void SafeDeque<T, blocking>::push_back(T&& item)
{
    std::lock_guard<std::mutex> lk(mLock);
    if constexpr (blocking){
        mCv.wait(lk, [&](){return mDq.size() < mSize});
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
T SafeDeque<T, blocking>::front() const
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.front();
}

template <typename T, bool blocking>
T SafeDeque<T, blocking>::back() const
{
    std::lock_guard<std::mutex> lk(mLock);
    return mDq.back();
}


} // namespace concurrency


} // namespace utils



