#include "MPPT.hpp"

MPPT::MPPT(MPPT_Algorithm algorithm, uint16_t lower_limit, uint16_t upper_limit, int16_t pwm_step, float current_max) {
  _mppt_algorithm = algorithm;
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;
  _pwm_step = pwm_step;
  _current_max = current_max;
}

void MPPT::set_voltage(float voltage) {
  _measurement.voltage = voltage;
}

void MPPT::set_current(float current) {
  _measurement.current = current;
}

void MPPT::set_power(float power) {
  _measurement.power = power;
}

void MPPT::set_algorithm(MPPT_Algorithm algorithm) {
  _mppt_algorithm = algorithm;
}

void MPPT::set_current_limit(float current_max) {
  _current_max = current_max;
}

void MPPT::update() {
  int16_t adjustment = 0;
  switch (_mppt_algorithm) {
    case PERTURB_AND_OBSERVE:
      adjustment = find_mppt_po(_last_measurement, _measurement);
      break;
    case INCREMENTAL_CONDUCTANCE:
      adjustment = find_mppt_ic(_last_measurement, _measurement);
      break;
  }
  _last_measurement = _measurement;
  if (_measurement.current >= _current_max) {
      adjustment = -pwm_step;
  }
  _measurement.duty_cycle = constrain((int32_t)_last_measurement.duty_cycle + adjustment, _lower_limit, _upper_limit);
}

void MPPT::reset(uint16_t duty_cycle = 15) {
  _last_measurement.duty_cycle = duty_cycle;
  _measurement.duty_cycle = duty_cycle;
}

uint16_t MPPT::get_duty_cycle() {
  return _measurement.duty_cycle;
}
float MPPT::get_voltage() {
  return _measurement.voltage;
}
float MPPT::get_current() {
  return _measurement.current;
}
float MPPT::get_power() {
  return _measurement.power;
}

int16_t MPPT::find_mppt_po(measurement_t last_measurement, measurement_t measurement) {
  if (measurement.power > last_measurement.power) {
    if (measurement.duty_cycle < last_measurement.duty_cycle) {
      return -_pwm_step;
    } else {
      return _pwm_step;
    }
  } else {
    if (measurement.duty_cycle > last_measurement.duty_cycle) {
      return -_pwm_step;
    } else {
      return _pwm_step;
    }
  }
  return 0;
}

int16_t MPPT::find_mppt_ic(measurement_t last_measurement, measurement_t measurement) {
  float dU = measurement.voltage - last_measurement.voltage;
  float dI = measurement.current - last_measurement.current;
  if (fabsf(dU) < 0.001f) {
    if (fabsf(dI) < 0.001f) {
      return 0;
    } else {
      if (dI > 0.0f) {
        return -_pwm_step;
      } else {
        return _pwm_step;
      }
    }
  } else {
    float incremental_conductance = dI / dU;
    float instantaneous_conductance = -measurement.current / measurement.voltage;
    if (fabsf(incremental_conductance - instantaneous_conductance) <= 0.001f) {
      return 0;
    } else if ((incremental_conductance - instantaneous_conductance) >= 0.001f) {
      return -_pwm_step;
    } else if ((instantaneous_conductance - incremental_conductance) >= 0.001f) {
      return _pwm_step;
    }
  }
  return 0;
}
