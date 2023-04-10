#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <utils/NonCopyable.hpp>

namespace utils
{

namespace logger
{
using namespace spdlog;

class Logger : public noncopyable::NonCopyable
{
private:
    
    std::shared_ptr<spdlog::logger> _lg;

    Logger()
    {
        _lg = spdlog::default_logger();
        _lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [td %t] %^[%l] %v%$");
    }

public:

    ~Logger() = default;

    void setLogLevel(level::level_enum lvl)
    {
        if(!_lg) getInstance();
        _lg->set_level(lvl);
    }

    void setLogFile(std::string fn, level::level_enum lvl=level::info)
    {
        _lg = spdlog::basic_logger_mt("logger", fn);
        _lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [td %t] %^[%l] %v%$");
        _lg->set_level(lvl);
        _lg->flush_on(lvl);
    }

    static std::shared_ptr<Logger> getInstance(){
        static auto lg = std::shared_ptr<Logger>(new Logger());
        return lg;
    }

    template<typename ...ARGS>
    void trace(ARGS ...args)    { _lg->trace(args ...); }

    template<typename ...ARGS>
    void debug(ARGS ...args)    { _lg->debug(args ...); }

    template<typename ...ARGS>
    void info(ARGS ...args)     { SPDLOG_LOGGER_INFO(_lg, args ...); }

    template<typename ...ARGS>
    void warn(ARGS ...args)     { SPDLOG_LOGGER_WARN(_lg, args ...); }

    template<typename ...ARGS>
    void error(ARGS ...args)    { SPDLOG_LOGGER_ERROR(_lg, args ...); }

};

} // namespace logger

    
} // namespace utils

