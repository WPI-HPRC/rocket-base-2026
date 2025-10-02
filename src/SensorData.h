#pragma once

/*
generic data thing

timestamp can get with no cast?

data can be looked up by "type"
getting acc data would be something like
SensorManager.getData(Acc) or something like it

generic? template

*/

template<typename T>
class SensorData {
  
};

template<typename T>
class Sensor {
  // private
  private:
    SensorData<T> data;
    // polling info
    
  public:
    T* getData();

    void poll();
    bool init();
    Sensor( )
    
    
};
