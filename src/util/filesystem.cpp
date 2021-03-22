#include "filesystem.h"

#include <iomanip>
#include <iostream>
#include <string>

namespace util {
fs::path PathFinder::assetPath;
bool PathFinder::initialize() {
    assetPath = fs::current_path();
    while (!fs::exists(assetPath / "assets/DO NOT remove this placeholder file") && assetPath.has_relative_path()) {
        assetPath = assetPath.parent_path();
    }
    if (!fs::exists(assetPath / "assets/DO NOT remove this placeholder file")) {
        return false;
    }
    assetPath /= "assets";
    std::cout << std::left << std::setw(26) << "Assets folder path"
              << ": " << assetPath.string() << std::endl;
    return true;
}
fs::path PathFinder::find(const char* filename) {
    if (assetPath.empty()) initialize();
    return assetPath / filename;
}
}  // namespace util
