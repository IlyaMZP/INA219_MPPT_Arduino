#include "MPPT.hpp"

MPPT::MPPT(MPPT_Algorithm algorithm, uint16_t lower_limit, uint16_t upper_limit, int16_t pwm_step, float current_max) {
  _mppt_algorithm = algorithm;
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;
  _pwm_step = pwm_step;
  _current_max = current_max;
}

void MPPT::set_algorithm(MPPT_Algorithm algorithm) {
  _mppt_algorithm = algorithm;
}

void MPPT::set_current_limit(float current_max) {
  _current_max = current_max;
}

void MPPT::set_measurement_callback(measurement_callback_t callback) {
  _measurement_callback = callback;
}

void MPPT::set_duty_cycle_callback(dc_callback_t callback) {
  _dc_callback = callback;
}

void MPPT::sweep(uint32_t sample_delay) {
  // FIXME: There is no current limit in sweep
  if (!_dc_callback) {
    // Sweep can only be done if we can set duty cycle.
    return;
  }
  uint16_t max_dc = 0;
  float max_p = 0.0f;
  measurement_t sweep_measurement;
  for (uint16_t dc = _lower_limit; dc <= _upper_limit; dc += _pwm_step) {
    _dc_callback(dc);
    delay(sample_delay);
    _measurement_callback(&sweep_measurement);
    if (sweep_measurement.power > max_p) {
      max_p = sweep_measurement.power;
      max_dc = dc;
      // Just so our mppt algorithm doesn't freak out when we return
      // Let's update the internal measurement state
      _measurement = sweep_measurement;
    }
  }

  _measurement.duty_cycle = max_dc;
  _last_measurement = _measurement;
  _dc_callback(_measurement.duty_cycle);
  // Wait for voltage to stabilize again before returning
  delay(sample_delay);
}

void MPPT::update() {
  if (!_measurement_callback) {
    return;
  }
  // I have trust issues
  uint16_t _saved_dc = _measurement.duty_cycle;
  _measurement_callback(&_measurement);
  _measurement.duty_cycle = _saved_dc;

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
      adjustment = -_pwm_step;
  }
  _measurement.duty_cycle = constrain((int32_t)_last_measurement.duty_cycle + adjustment, _lower_limit, _upper_limit);
  if (_dc_callback) {
    _dc_callback(_measurement.duty_cycle);
  }
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
