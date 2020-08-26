#ifndef ValueSmoothing_h
#define ValueSmoothing_h
#include <Arduino.h>

class SmoothedValue
{
public:
    SmoothedValue(int num_Readings, int pin);  // Konstruktor
    int calculateWithNewValue();


private:
    int _pin;
    int _num_readings;
    int *_readings;
    int _read_index;
    int _total;
};

#endif