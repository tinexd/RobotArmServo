#include <Arduino.h>
#include "ValueSmoothingLib.h"
#include "VarSpeedServo.h" // Lib-Ordner von github von Hand kopieren nach:
                           // E:\.......\RobotArmServo\.pio\libdeps\uno\VarSpeedServo
                           // weil VarSpeedServo-Lib über PlatformIO nicht für Arduino ist

VarSpeedServo servo_hip;      // create servo object to control a servo
VarSpeedServo servo_shoulder; // create servo object to control a servo
VarSpeedServo servo_elbow;    // create servo object to control a servo
VarSpeedServo servo_finger;   // create servo object to control a servo

const int poti_pin_hip = A0;
const int poti_pin_shoulder = A1;
const int poti_pin_elbow = A2;
const int poti_pin_finger = A3;

const int save_button_pin = 10;
const int play_button_pin = 11;
const int stop_button_pin = 2;   // Interrupt-Pin

int hip_min_pos = 0, hip_max_pos = 175, hip_target_pos;
int shoulder_min_pos = 80, shoulder_max_pos = 150, shoulder_target_pos;
int elbow_min_pos = 30, elbow_max_pos = 120, elbow_target_pos;
int finger_min_pos = 35, finger_max_pos = 135, finger_target_pos;
int hip_speed = 20;      // 0=full speed, 1-255 slower to faster
int shoulder_speed = 25; // 0=full speed, 1-255 slower to faster
int elbow_speed = 25;    // 0=full speed, 1-255 slower to faster
int finger_speed = 60;  // 0=full speed, 1-255 slower to faster

int hip_actual_pos, shoulder_actual_pos, elbow_actual_pos, finger_actual_pos;
int *saved_positions = NULL;
int saved_positions_count = 0;

int num_readings = 15;
#define DEBOUNCE_TIME 100ul

volatile bool isStopped = LOW;

SmoothedValue hipPotiValue(num_readings, poti_pin_hip); // Anzahl der Durchläufe, Pin
SmoothedValue shoulderPotiValue(num_readings, poti_pin_shoulder);
SmoothedValue elbowPotiValue(num_readings, poti_pin_elbow);
SmoothedValue fingerPotiValue(num_readings, poti_pin_finger);

bool SaveButtonIsPressed(int pin, unsigned long millis_now, unsigned long debounce_time);
bool PlayButtonIsPressed(int pin, unsigned long millis_now, unsigned long debounce_time);
void IsrFunction();

void setup()
{
  Serial.begin(9600);
  servo_hip.attach(3);      // attaches the servo on pin 9 to the servo object
  delay(500);               // weil alle Servos auf einmal zu viel Strom ziehen würden
  servo_shoulder.attach(5); // attaches the servo on pin 9 to the servo object
  delay(500);               // weil alle Servos auf einmal zu viel Strom ziehen würden
  servo_elbow.attach(6);    // attaches the servo on pin 9 to the servo object
  delay(500);               // weil alle Servos auf einmal zu viel Strom ziehen würden
  servo_finger.attach(9);   // attaches the servo on pin 9 to the servo object
  delay(500);               // weil alle Servos auf einmal zu viel Strom ziehen würden

  pinMode(poti_pin_hip, INPUT);
  pinMode(poti_pin_shoulder, INPUT);
  pinMode(poti_pin_elbow, INPUT);
  pinMode(poti_pin_finger, INPUT);
  pinMode(save_button_pin, INPUT_PULLUP);
  pinMode(play_button_pin, INPUT_PULLUP);
  pinMode(stop_button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stop_button_pin), IsrFunction, FALLING); // 0,1 µF Kondensator an Pin und GND, da sonst bei Play durch Störungen der Motoren der Interrupt triggert

  for (int i = 0; i < num_readings; i++)
  {
    hip_target_pos = map(hipPotiValue.calculateWithNewValue(), 0, 1023, hip_min_pos, hip_max_pos);
    shoulder_target_pos = map(shoulderPotiValue.calculateWithNewValue(), 0, 1023, shoulder_min_pos, shoulder_max_pos);
    elbow_target_pos = map(elbowPotiValue.calculateWithNewValue(), 0, 1023, elbow_min_pos, elbow_max_pos);
    finger_target_pos = map(fingerPotiValue.calculateWithNewValue(), 1023, 0, finger_min_pos, finger_max_pos);
  }
}

void loop()
{
  unsigned long millis_now = millis();
  hip_target_pos = map(hipPotiValue.calculateWithNewValue(), 0, 1023, hip_min_pos, hip_max_pos);
  shoulder_target_pos = map(shoulderPotiValue.calculateWithNewValue(), 0, 1023, shoulder_min_pos, shoulder_max_pos);
  elbow_target_pos = map(elbowPotiValue.calculateWithNewValue(), 0, 1023, elbow_min_pos, elbow_max_pos);
  finger_target_pos = map(fingerPotiValue.calculateWithNewValue(), 1023, 0, finger_min_pos, finger_max_pos);

  servo_hip.write(hip_target_pos, hip_speed, true);                // tell servo to go to position in variable 'pos'
  servo_shoulder.write(shoulder_target_pos, shoulder_speed, true); // tell servo to go to position in variable 'pos'
  servo_elbow.write(elbow_target_pos, elbow_speed, true);          // tell servo to go to position in variable 'pos'
  servo_finger.write(finger_target_pos, finger_speed, true);       // tell servo to go to position in variable 'pos'

  // Serial.println(hip_target_pos);
  // Serial.println(shoulder_target_pos);
  // Serial.println(elbow_target_pos);
  // Serial.println(finger_target_pos);
  // Serial.println();
  // delay(100);

  if (SaveButtonIsPressed(save_button_pin, millis_now, DEBOUNCE_TIME))
  {
    saved_positions_count++;
    if (saved_positions == NULL)
    {
      saved_positions = (int *)calloc(4 * saved_positions_count, sizeof(int)); // calloc initialisiert mit 0; 4 * weil immer 4 Poti-Werte gespeichert werden müssen
      if (saved_positions == NULL)
        Serial.println("Speicher reservieren nicht möglich!");
      else
        Serial.println("Speicher reserviert!");
    }
    else
    {
      saved_positions = (int *)realloc(saved_positions, 4 * saved_positions_count * sizeof(int)); // 4 * weil immer 4 Poti-Werte gespeichert werden müssen
      if (saved_positions == NULL)
        Serial.println("Speicher erweitern nicht möglich!");
      else
        Serial.println("Speicher erweitert!");
    }

    saved_positions[4 * saved_positions_count - 4] = hip_target_pos;      // für ersten Durchlauf -> 4 * 1 - 4 = 0 -> Postion 0 im Array
    saved_positions[4 * saved_positions_count - 3] = shoulder_target_pos; // für ersten Durchlauf -> 4 * 1 - 3 = 1 -> Postion 1 im Array
    saved_positions[4 * saved_positions_count - 2] = elbow_target_pos;    // für ersten Durchlauf -> 4 * 1 - 2 = 2 -> Postion 2 im Array
    saved_positions[4 * saved_positions_count - 1] = finger_target_pos;   // für ersten Durchlauf -> 4 * 1 - 1 = 3 -> Postion 3 im Array
  }

  if (PlayButtonIsPressed(play_button_pin, millis_now, DEBOUNCE_TIME))
  {
    Serial.println("Play!");
    while (!isStopped)
    {
      for (int i = 0; i < saved_positions_count; i++)
      {
        servo_hip.write(saved_positions[i * 4], hip_speed, true); // tell servo to go to position in variable 'pos'
        if (isStopped)
          break;
        servo_shoulder.write(saved_positions[i * 4 + 1], shoulder_speed, true); // tell servo to go to position in variable 'pos'
        if (isStopped)
          break;
        servo_elbow.write(saved_positions[i * 4 + 2], elbow_speed, true); // tell servo to go to position in variable 'pos'
        if (isStopped)
          break;
        servo_finger.write(saved_positions[i * 4 + 3], finger_speed, true); // tell servo to go to position in variable 'pos'
      }
    }
  }

  if (isStopped)
  {
    free(saved_positions);
    saved_positions = NULL;
    saved_positions_count = 0;
    Serial.println("Stopp und Speicher gelöscht!");
    isStopped = LOW;
  }
}

bool SaveButtonIsPressed(int pin, unsigned long millis_now, unsigned long debounce_time)
{
  static unsigned long last_millis_button = 0;
  static bool button_was_pushed = LOW;
  bool button_is_pushed = !digitalRead(pin); // betätigt ist LOW

  if (button_is_pushed && !button_was_pushed && millis_now - last_millis_button >= DEBOUNCE_TIME)
  { // wenn fallende Flanke erkannt wurde und die Entprellzeit abgelaufen ist, wird der Tastendruck weiterverarbeitet
    last_millis_button = millis_now;
    button_was_pushed = button_is_pushed;
    return 1;
  }
  else
  {
    button_was_pushed = button_is_pushed;
    return 0;
  }
}

bool PlayButtonIsPressed(int pin, unsigned long millis_now, unsigned long debounce_time)
{
  static unsigned long last_millis_button = 0;
  static bool button_was_pushed = LOW;
  bool button_is_pushed = !digitalRead(pin); // betätigt ist LOW

  if (button_is_pushed && !button_was_pushed && millis_now - last_millis_button >= DEBOUNCE_TIME)
  { // wenn fallende Flanke erkannt wurde und die Entprellzeit abgelaufen ist, wird der Tastendruck weiterverarbeitet
    last_millis_button = millis_now;
    button_was_pushed = button_is_pushed;
    return 1;
  }
  else
  {
    button_was_pushed = button_is_pushed;
    return 0;
  }
}

void IsrFunction()
{
  isStopped = HIGH;
}