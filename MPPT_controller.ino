#include <Wire.h>
#include <Adafruit_INA219.h>
#include "MPPT.hpp"

Adafruit_INA219 ina219;

MPPT mppt(MPPT::INCREMENTAL_CONDUCTANCE);

#define power_min_mw 50  // Minimum power level before reset

uint32_t power_good_timestamp = 0;
double energy_mJ = 0;
uint32_t last_time;
uint32_t current_time;

void measurement_callback(MPPT::measurement_t* const measurement) {
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V()*1000.0f; // Vin- voltage
  measurement->power = ina219.getPower_mW();
  measurement->current = ina219.getCurrent_mA();
  measurement->voltage = busvoltage + shuntvoltage; // Vin+ voltage
}

void dc_callback(uint16_t duty_cycle) {
  // Optional callback function for setting the duty cycle
  // Needed for mppt.sweep()
  analogWrite(9, duty_cycle);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  ina219.begin();
  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  mppt.set_current_limit(1500.0f);
  mppt.set_measurement_callback(&measurement_callback);
  mppt.set_duty_cycle_callback(&dc_callback);
  current_time = millis();
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
    if (c == 's') {
      mppt.sweep();
    }
  }
  // put your main code here, to run repeatedly:
  if (mppt.get_power() >= power_min_mw) {
      power_good_timestamp = millis();
  }

  last_time = current_time;
  current_time = millis();
  // Not gonna be precise
  energy_mJ += mppt.get_power() * (current_time-last_time) / 1000.0;
  mppt.update();

  Serial.print(mppt.get_current());
  Serial.print(",");
  Serial.print(mppt.get_voltage());
  Serial.print(",");
  Serial.print(energy_mJ/3600.0); // mWh
  Serial.print(",");
  Serial.println(mppt.get_power());
  delay(50);
  if (millis() - power_good_timestamp >= 10000) { // Sweep if we didn't have good power
      mppt.sweep();                               // for more than 10 seconds
      power_good_timestamp = millis();
  }
}
