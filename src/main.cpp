#include <Arduino.h>
#include <pins_arduino.h>
#include <mbed.h>
#include <rtos.h>
#include <PinNames.h>
#include <mbed_wait_api.h>
#include <mbed_events.h>

//mbed::PwmOut d2(P1_11);
mbed::Timer steeringPulseTimer;
mbed::InterruptIn steeringIn(P1_11);
rtos::Semaphore in_steering_sem(1);
events::EventQueue queue(32 * EVENTS_EVENT_SIZE);
rtos::Thread pwmInCallbackThread;


int32_t in_steering_pulse_start;
int32_t in_steering_pulse_end;
int32_t in_steering_pulse_width;
int32_t in_steering_update_ok;
int32_t in_steering_update_failed;

void steering_rise_isr(void);
void steering_fall_isr(void);
void steering_fall_uc(int width);

void setup() {
  Serial.begin(9600);
  steeringPulseTimer.reset();
  // Setup pin D2 (P1_11) to receive steering input
  steeringIn.rise(mbed::callback(steering_rise_isr));
  steeringIn.fall(mbed::callback(steering_fall_isr));
  in_steering_update_ok = 0;
  in_steering_update_failed = 0;
  pwmInCallbackThread.start(mbed::callback(&queue,&events::EventQueue::dispatch_forever));
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


void steering_rise_isr(void) {
  steeringPulseTimer.reset();
  steeringPulseTimer.start();
}


void steering_fall_isr(void) {
  steeringPulseTimer.stop();
  queue.call(steering_fall_uc, steeringPulseTimer.read_us());
}

void steering_fall_uc(int width) {
  bool acq = in_steering_sem.try_acquire_for(2);
  if (acq) {
    in_steering_pulse_width = width;
    in_steering_sem.release();
    in_steering_update_ok++;
  } else in_steering_update_failed++;
}

void loop() {
  Serial.print("Pulse: ");
  in_steering_sem.acquire();
  Serial.print(in_steering_pulse_width);
  Serial.print("\tacqFails: ");
  Serial.println(in_steering_update_failed);
  in_steering_sem.release();
  wait_us(200000);
}