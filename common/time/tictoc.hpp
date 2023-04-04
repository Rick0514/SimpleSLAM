#pragma once

#include <spdlog/fmt/fmt.h>
#include <chrono>

// tictoc support for spdlog  (using std::chrono::steady_clock).
// Displays elapsed seconds since construction as double.
//
// Usage:
//
// spdlog::tictoc sw;
// ...
// spdlog::debug("Elapsed: {} seconds", sw);    =>  "Elapsed 0.005116733 seconds"
// spdlog::info("Elapsed: {:.6} seconds", sw);  =>  "Elapsed 0.005163 seconds"
//
//
// If other units are needed (e.g. millis instead of double), include "fmt/chrono.h" and use "duration_cast<..>(sw.elapsed())":
//
// #include <spdlog/fmt/chrono.h>
//..
// using std::chrono::duration_cast;
// using std::chrono::milliseconds;
// spdlog::info("Elapsed {}", duration_cast<milliseconds>(sw.elapsed())); => "Elapsed 5ms"

namespace common
{
namespace time
{
class tictoc
{
    using clock = std::chrono::steady_clock;
    std::chrono::time_point<clock> start_tp_;

public:
    tictoc()
        : start_tp_{clock::now()}
    {}

    std::chrono::duration<double> elapsed() const
    {
        return std::chrono::duration<double>(clock::now() - start_tp_);
    }

    void tic()
    {
        start_tp_ = clock::now();
    }
};
    

} // namespace time
}

template<>
struct fmt::formatter<common::time::tictoc> : fmt::formatter<double>
{
    template<typename FormatContext>
    auto format(const common::time::tictoc &tt, FormatContext &ctx) -> decltype(ctx.out())
    {
        return formatter<double>::format(tt.elapsed().count(), ctx);
    }
};