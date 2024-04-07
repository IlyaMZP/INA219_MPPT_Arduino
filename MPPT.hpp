#include <stdint.h>
#include <Arduino.h>

class MPPT {
public:
  struct measurement_t {
    float voltage = 0.0f;
    float current = 0.0f;
    float power = 0.0f;
    uint16_t duty_cycle = 0;
  };
  enum MPPT_Algorithm { PERTURB_AND_OBSERVE, INCREMENTAL_CONDUCTANCE };

  MPPT(MPPT_Algorithm agorithm = PERTURB_AND_OBSERVE, uint16_t lower_limit = 2, uint16_t upper_limit = 180, int16_t pwm_step = 1, float current_max = 3200.0f);
  void set_voltage(float voltage);
  void set_current(float current);
  void set_power(float power);
  void set_algorithm(MPPT_Algorithm algorithm);
  void set_current_limit(float current_max);

  void update();
  void reset(uint16_t duty_cycle = 15);

  uint16_t get_duty_cycle();
  float get_voltage();
  float get_current();
  float get_power();
private:
  measurement_t _measurement;
  measurement_t _last_measurement;
  uint16_t _lower_limit;
  uint16_t _upper_limit;
  uint16_t _pwm_step;
  float _current_max;
  MPPT_Algorithm _mppt_algorithm;
  int16_t find_mppt_po(measurement_t last_measurement, measurement_t measurement);
  int16_t find_mppt_ic(measurement_t last_measurement, measurement_t measurement);
};
