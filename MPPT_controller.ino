#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

#define upper_limit 180  // Upper PWM limit for boost converter
#define lower_limit 2    // Lower PWM limit  for boost converter
#define power_min_mw 50  // Minimum power level before reset
#define mppt_function find_mppt_po // find_mppt_po (Perturb and observe) or find_mppt_ic (Incremental conductance)

struct measurement_t {
  float voltage;
  float current;
  float power;
  uint16_t duty_cycle;
};

uint32_t power_good_timestamp = 0;
measurement_t measurement;
measurement_t last_measurement;

void setup() {
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  ina219.begin();
  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  measurement.duty_cycle = 16;
}


measurement_t measure(measurement_t measurement) {
  float shuntvoltage = ina219.getShuntVoltage_mV(); 
  float busvoltage = ina219.getBusVoltage_V()*1000.0f; // Vin- voltage
  measurement.power = ina219.getPower_mW();
  measurement.current = ina219.getCurrent_mA();
  measurement.voltage = busvoltage + shuntvoltage; // Vin+ voltage
  return measurement;
}

void loop() {
  if (measurement.power >= power_min_mw) {
      power_good_timestamp = millis();
  }
  measurement = measure(measurement);
  int16_t adjustment = mppt_function(last_measurement, measurement);
  last_measurement = measurement;
  measurement.duty_cycle = constrain(last_measurement.duty_cycle+adjustment, lower_limit, upper_limit);
  
  analogWrite(9,measurement.duty_cycle);
  Serial.print(measurement.duty_cycle);
  Serial.print(",");
  Serial.print(measurement.current);
  Serial.print(",");
  Serial.print(measurement.voltage);
  Serial.print(",");
  Serial.println(measurement.power);
  delay(50);
  #if 1 // janky current limit for testing
  if (measurement.current > 1500.0f) {
    if (measurement.duty_cycle >= 2) {
      measurement.duty_cycle -= 2;
    }
  }
  #endif
  if (millis() - power_good_timestamp >= 10000) { // Reset if we didn't have good power
      measurement.duty_cycle = 15;                             // for more than 10 seconds
      power_good_timestamp = millis();
  }
}

#define pwm_step 1

// Perturb and observe
int16_t find_mppt_po(measurement_t last_measurement, measurement_t measurement) {
  if (measurement.power > last_measurement.power) {
    if (measurement.duty_cycle < last_measurement.duty_cycle) {
      return -pwm_step;
    } else {
      return pwm_step;
    }
  } else {
    if (measurement.duty_cycle > last_measurement.duty_cycle) {
      return -pwm_step;
    } else {
      return pwm_step;
    }
  }
  return 0;
}

// Incremental conductance
int16_t find_mppt_ic(measurement_t last_measurement, measurement_t measurement) {
  float dU = measurement.voltage - last_measurement.voltage;
  float dI = measurement.current - last_measurement.current;
  if (fabsf(dU) < 0.001f) {
    if (fabsf(dI) < 0.001f) {
      return 0;
    } else {
      if (dI > 0.0f) {
        return -pwm_step;
      } else {
        return pwm_step;
      }
    }
  } else {
    float incremental_conductance = dI/dU;
    float instantaneous_conductance = -measurement.current/measurement.voltage;
    if (fabsf(incremental_conductance + instantaneous_conductance) <= 0.001f) {
      return 0;
    } else if ((incremental_conductance - instantaneous_conductance) >= 0.001f) {
      return -pwm_step;
    } else if ((instantaneous_conductance - incremental_conductance) >= 0.001f) {
      return pwm_step;
    }
  }
}
