#pragma once

#include <chrono>
#include <cstdint>

namespace util {
class Clock final {
 public:
    void reset();
    double timeElapsed();

 private:
    using clock = std::chrono::steady_clock;
    using unit = std::chrono::milliseconds;
    using ns = std::chrono::nanoseconds;
    constexpr static int64_t baseUnit = std::chrono::duration_cast<ns>(unit(1)).count();
    clock::time_point last;
};

}  // namespace util
