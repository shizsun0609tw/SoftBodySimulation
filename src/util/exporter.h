#pragma once
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
namespace util {
class Writer {
 private:
    static std::atomic<int> pictureCounter;
    static std::atomic<int> filenameFormattingThreads;

    std::condition_variable cvNew, cvFinish;
    std::mutex notifyNew, bufferLock;

    std::vector<unsigned char> imageBuffer;
    std::atomic<bool> stop, hasData;
    std::thread writerThread;
    void infiniteLoop(int width, int height);

 public:
    Writer(int width, int height);
    ~Writer();
    static int getPictureCounter();
    static void resetPictureCounter();
    void write();
    void readCurrentPBO();
    void join();
};
class Exporter final {
 private:
    Exporter(const Exporter&) = delete;
    Exporter& operator=(const Exporter&) = delete;
    Exporter(Exporter&&) = delete;

    int width = 0, height = 0;
    int threadCount = std::thread::hardware_concurrency() - 1;
    int currentIndex = 0;
    int currentWriter = 0;
    unsigned int pixelBuffer[2] = {};
    std::vector<std::unique_ptr<Writer>> writer;

 public:
    Exporter();
    ~Exporter();
    void resize(int screenWidth, int screenHeight);
    void outputScreenShot();
};

}  // namespace util
