#include "clock.h"

namespace util {
void Clock::reset() { last = clock::now(); }

double Clock::timeElapsed() {
    double elapsed = static_cast<double>(std::chrono::duration_cast<ns>(clock::now() - last).count());
    last = clock::now();
    return elapsed / baseUnit;
}
}  // namespace util
