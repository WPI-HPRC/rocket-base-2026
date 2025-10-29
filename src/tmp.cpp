#include <Arduino.h>

enum SensorDataType : uint8_t {
  DATA_TYPE_IMU_1 = 0,
  DATA_TYPE_IMU_2 = 1,
};

struct SensorDataDescriptor {
  SensorDataType type;
  size_t size;
  unsigned long time_stamp;
};

// base for sensors
template <class Derived>
class SensorBase {
public:
  // some way to get size
  // init
  // poll_rate
  inline void init() {
    return static_cast<Derived*>(this)->init_impl();
  }

  inline long poll_rate() const {
    return static_cast<Derived*>(this)->poll_rate_impl();
  }

  // something for data size
  // something for data type
  
};

// might want helpers for allocating and stuff?
// want some way to find what the larges allocation will be

template <class... Sensors>
// can have more than one senor as the template
class SensorManager {
  public:
    static size_t NUM = sizeof...(Sensors);
    // largest data packet?
    // list of sensors we have
  
    SensorManager(Sensors*... sensors) : sensors_(sensors...) {
      for(size_t i = 0; i < NUM; i++) {
        // some form of setup
      }    
    }

    void sensors_init() {
      for(size_t i = 0; i < NUM; i++) {
        
      }
    }
};
