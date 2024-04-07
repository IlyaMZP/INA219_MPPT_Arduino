#include <Wire.h>
#include <Adafruit_INA219.h>
#include "MPPT.hpp"

Adafruit_INA219 ina219;

MPPT mppt(MPPT::INCREMENTAL_CONDUCTANCE);

#define power_min_mw 50  // Minimum power level before reset

uint32_t power_good_timestamp = 0;
double energy_J = 0;
uint32_t last_time;
uint32_t current_time;

void setup() {
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  ina219.begin();
  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  mppt.set_current_limit(1500.0f);
}

void loop() {
  if (Serial.available() != 0) {
    char c = Serial.read();
    if (c == 'p') {
      mppt.set_algorithm(MPPT::PERTURB_AND_OBSERVE);
    }
    if (c == 'i') {
      mppt.set_algorithm(MPPT::INCREMENTAL_CONDUCTANCE);
    }
  }
  // put your main code here, to run repeatedly:
  if (mppt.get_power() >= power_min_mw) {
      power_good_timestamp = millis();
  }

  measure();
  mppt.update();
  
  analogWrite(9,mppt.get_duty_cycle());
  Serial.print(mppt.get_duty_cycle());
  Serial.print(",");
  Serial.print(mppt.get_current());
  Serial.print(",");
  Serial.print(mppt.get_voltage());
  Serial.print(",");
  Serial.println(mppt.get_power());
  delay(50);
  if (millis() - power_good_timestamp >= 10000) { // Reset if we didn't have good power
      mppt.reset();                               // for more than 10 seconds
      power_good_timestamp = millis();
  }
}

void measure() {
  float shuntvoltage = ina219.getShuntVoltage_mV(); 
  float busvoltage = ina219.getBusVoltage_V()*1000.0f; // Vin- voltage
  mppt.set_power(ina219.getPower_mW());
  mppt.set_current(ina219.getCurrent_mA());
  mppt.set_voltage(busvoltage + shuntvoltage); // Vin+ voltage
}
