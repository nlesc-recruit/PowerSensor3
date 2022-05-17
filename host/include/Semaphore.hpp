#pragma once

#include <condition_variable>
#include <mutex>

namespace PowerSensor {

/**
 * @brief A simple semaphore to hold/release lock when starting a new IO thread
 *
 */
class Semaphore {
 public:
    /**
     * @brief Construct a new Semaphore object
     *
     * @param initialLevel
     */
    explicit Semaphore(unsigned initialLevel = 0): level(initialLevel) {}

    /**
     * @brief Release lock
     *
     * @param count
     */
    void up(unsigned count = 1) {
      std::unique_lock<std::mutex> lock(mutex);
      level += count;
      if (count == 1)
        cv.notify_one();
      else
        cv.notify_all();
    }

    /**
     * @brief Obtain lock
     *
     * @param count
     */
    void down(unsigned count = 1) {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [this, count] { return level >= count; });
      level -= count;
    }

 private:
    std::mutex mutex;
    std::condition_variable cv;
    unsigned level;
};

}  // namespace PowerSensor
