#pragma once

#include <condition_variable>
#include <mutex>


namespace PowerSensor {


class Semaphore {
  public:
    Semaphore(unsigned initialLevel = 0): level(initialLevel) {};

    void up(unsigned count = 1) {
      std::unique_lock<std::mutex> lock(mutex);
      level += count;

      if (count == 1)
      	cv.notify_one();
      else
      	cv.notify_all();
    }

    void down(unsigned count = 1) {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [this, count] { return level >= count; });
      level -= count;
    }

  private:
    std::mutex		    mutex;
    std::condition_variable cv;
    unsigned		    level;
};

}

