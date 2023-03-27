#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <utils/NonCopyable.hpp>

#include <memory>

namespace utils
{

namespace logger
{

class Logger : public noncopyable::NonCopyable
{
private:
    
    std::shared_ptr<spdlog::logger> _lg;

    Logger()
    {
#ifdef LOG_FILE
        _lg = spdlog::basic_logger_mt("logger", LOG_FILE);
#else
        _lg = spdlog::default_logger();
#endif

#ifdef SPDLOG_ACTIVE_LEVEL
        _lg->set_level(static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));
#endif

        _lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [td %t] [%s:%#] %^[%l] %v%$");
    }

public:

    ~Logger() = default;

    static std::shared_ptr<Logger> getInstance(){
        static auto lg = std::shared_ptr<Logger>(new Logger());
        return lg;
    }

    template<typename ...ARGS>
    void trace(ARGS ...args)    { SPDLOG_LOGGER_TRACE(_lg, args ...); }

    template<typename ...ARGS>
    void info(ARGS ...args)     { SPDLOG_LOGGER_INFO(_lg, args ...); }

    template<typename ...ARGS>
    void warn(ARGS ...args)     { SPDLOG_LOGGER_WARN(_lg, args ...); }

    template<typename ...ARGS>
    void error(ARGS ...args)    { SPDLOG_LOGGER_ERROR(_lg, args ...); }

};

} // namespace logger

    
} // namespace utils

