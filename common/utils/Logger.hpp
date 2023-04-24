#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/bundled/ranges.h>
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
    std::atomic_bool _exit;

    Logger() : _exit(false)
    {
        _lg = spdlog::default_logger();
        _lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [td %t] %^[%l] %v%$");
    }

public:
    using Ptr = std::shared_ptr<Logger>;

    ~Logger() = default;

    void setLogLevel(level::level_enum lvl)
    {
        if(!_lg) getInstance();
        _lg->set_level(lvl);
    }

    void setLogFile(std::string fn, level::level_enum lvl=level::info)
    {
        _lg = spdlog::basic_logger_mt("logger", fn, true);
        _lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [td %t] %^[%l] %v%$");
        _lg->set_level(lvl);
        _lg->flush_on(lvl);
    }

    void exitProgram() {
        _lg->info("exit program!!");
        _exit.store(true);
    }

    bool isProgramExit() { return _exit.load(); }

    static Ptr getInstance(){
        static auto lg = std::make_shared<Logger>();
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

