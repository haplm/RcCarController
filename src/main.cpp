#include <Arduino.h>
#include <pins_arduino.h>
#include <mbed.h>
#include <rtos.h>
#include <PinNames.h>
#include <mbed_wait_api.h>

//mbed::PwmOut d2(P1_11);
mbed::Timer steeringPulseTimer;
mbed::Timer steeringCycleTimer;
mbed::InterruptIn steeringIn(P1_11);
rtos::Semaphore in_steering_sem(1);

int32_t in_steering_pulse_start;
int32_t in_steering_pulse_end;
int32_t in_steering_pulse_width;
int32_t in_steering_cycle_start;
int32_t in_steering_cycle_end;
int32_t in_steering_cycle_width;
int   in_steering_freq;

void steering_rise(void);
void steering_fall(void);

void setup() {
  Serial.begin(9600);
  steeringPulseTimer.reset();
  steeringCycleTimer.reset();
  in_steering_freq = 0;
  // Setup pin D2 (P1_11) to receive steering input
  steeringIn.rise(mbed::callback(steering_rise));
  steeringIn.fall(mbed::callback(steering_fall));
  // Setup pin D3 (P1_12) to receive throttle input
}

/* void setup_pwm_out_test() {
  d2.period_ms(200);
  d2.pulsewidth_us(1000);
  Serial.in_steering_pulse_start(9600);
  Serial.println(LED1);
} */

/* void pwm_out_test() {
  int pulsewidth = 1000;
  while(1) {
    pulsewidth += 10;
    d2.pulsewidth_us(pulsewidth);
    wait(0.1);
    if(pulsewidth >= 2000) {
        pulsewidth = 1000;
    }
    Serial.print(d2.read());
  }
} */

/**
 *  Interrupt pin rising edge interrupt handler. Reset and start steeringPulseTimer
 */
void steering_rise(void) {
  steeringCycleTimer.stop();
  bool acq = in_steering_sem.try_acquire();
  if (acq) {
    in_steering_cycle_width = steeringCycleTimer.read_us();
    in_steering_sem.release();
  }
  steeringPulseTimer.reset();
  steeringCycleTimer.reset();
  steeringPulseTimer.start();
  steeringCycleTimer.start();
  //in_steering_pulse_start = steeringPulseTimer.read_us();
}

/**
 *  Interrupt pin falling edge interrupt handler. Read and disengage steeringPulseTimer.
 *  Calculate raw echo pulse length
 */
void steering_fall(void) {
  //in_steering_pulse_end = steeringPulseTimer.read_us();
  steeringPulseTimer.stop();
  bool acq = in_steering_sem.try_acquire();
  if (acq) {
    in_steering_pulse_width = steeringPulseTimer.read_us();
    in_steering_sem.release();
  }
  //in_steering_pulse_width = in_steering_pulse_end - in_steering_pulse_start;
}

void loop() {
  Serial.print("Pulse: ");
  in_steering_sem.acquire();
  Serial.print(in_steering_pulse_width);
  Serial.print("\tCycle: ");
  Serial.print(in_steering_cycle_width);
  if (in_steering_cycle_width > 0) {
    in_steering_freq = (1.0 / (float)in_steering_cycle_width) * 1000.0 * 1000.0;
  }
  in_steering_sem.release();
  Serial.print("\tFreq: ");
  Serial.println(in_steering_freq);
  wait_us(200000);
}