#pragma once

// copy from boost::noncopyable

namespace utils
{
namespace noncopyable
{

class NonCopyable
{
protected:
    NonCopyable(){}
    ~NonCopyable(){}

private:
    NonCopyable(const NonCopyable&);
    const NonCopyable& operator=(const NonCopyable&);
};

} // namespace noncopyable


} // namespace utils
