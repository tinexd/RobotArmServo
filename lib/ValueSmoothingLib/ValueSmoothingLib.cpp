#include <Arduino.h>
#include "ValueSmoothingLib.h"
/*

based on:

  Smoothing

  Reads repeatedly from an analog input, calculating a running average and
  printing it to the computer. Keeps ten readings in an array and continually
  averages them.

  The circuit:
  - analog sensor (potentiometer will do) attached to analog input 0

  created 22 Apr 2007
  by David A. Mellis  <dam@mellis.org>
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Smoothing
*/

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.

SmoothedValue::SmoothedValue(int num_readings, int pin) // Konstruktor
{
  _num_readings = num_readings;
  _pin = pin;
  _read_index = 0;
  _total = 0;
  _readings = NULL;
  _readings = (int *)calloc(_num_readings, sizeof(int)); // calloc initialisiert mit 0
  if (_readings == NULL)
    Serial.println("Speicher reservieren nicht möglich! (Glättung)");
  else
    Serial.println("Speicher reserviert! (Glättung)");
}

int SmoothedValue::calculateWithNewValue()
{
  // subtract the last reading:
  _total = _total - _readings[_read_index];
  // read from the sensor:
  _readings[_read_index] = analogRead(_pin);

  // add the reading to the total:
  _total = _total + _readings[_read_index];
  // advance to the next position in the array:
  _read_index++;
  // if we're at the end of the array...
  if (_read_index >= _num_readings)
  {
    // ...wrap around to the beginning:
    _read_index = 0;
  }
  // calculate the average:
  return _total / _num_readings;
}
