#pragma once

class Debouncer {
  public:
    Debouncer(long timeHigh) : timeHigh(timeHigh) {}

    bool update(bool val, long now) {
      if (!val) {
        lastTime = -1;
        return false;
      } else {
        if (lastTime == -1) lastTime = now;
        if (now - lastTime >= timeHigh) {
          return true;
        } else {
          return false;
        }
      }
    }

  private:
    long timeHigh;
    long lastTime = 0;
};