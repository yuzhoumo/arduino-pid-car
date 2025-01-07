#include <util/atomic.h>

/* ------------------ PIN LABELS ------------------------------------------- */

#define ENCODER_SIG 2
#define MOTOR_INA1  3
#define MOTOR_INA2  4
#define MOTOR_ENA   5
#define START_BTN   9

// uncomment to enable logging
#define DEBUG_MODE

/* ------------------ INPUT SETTINGS --------------------------------------- */

// offset distance in mm from the front of car to axel
const float mm_offset = 20;

// target distance in millimeters
const float mm_target_distance = 10000.0 - mm_offset;

// distance to end the speedup phase (peg the motor to 100% or cruise at max vel)
const float mm_speedup_distance = 4000.0;

// max target velocity in mm/s
const float max_vel = 3000.0;

// slowdown factor
const float slowdown_accel = 600.0;

// Kp gain (we don't use Ki, Kd)
const float Kp = 40.0;

// number of milliseconds to brake (run the motor in reverse direction)
const unsigned int ms_brake_time = 100;

// control loop interval (ms)
const unsigned long control_interval = 50;

// distance travelled in millimeters per encoder tick
// wheel circumference in mm / encoder ticks per wheel rotation
// should determine this experimentally based on logs
// divide number of ticks by actual distance travelled
// the ideal (60.0 * PI) / 45.0 was slightly off (~4.18)
// the following number we got by logging encoder ticks and
// dividing by actual distance travelled in a test run (after
// compensating for front of car to axel distance)
const float mm_per_tick = 4.24384;

/* ------------------ PRE-COMPUTED CONSTANTS ------------------------------- */

// number of ticks at which the speedup phase ends
const unsigned int speedup_ticks = (unsigned int)(mm_speedup_distance / mm_per_tick);

// number of encoder ticks to reach the distance at which we trigger braking
const unsigned int target_ticks = (unsigned int)(mm_target_distance / mm_per_tick);

/* ------------------ GLOBAL STATE VARIABLES ------------------------------- */

bool is_started = false; // track if the car is started
unsigned long last_control_time  = 0; // last time we updated velocity PID (ms)
unsigned long last_encoder_ticks = 0; // tick count at previous cycle
unsigned long start_time = 0;

/* ------------------ ENCODER INTERRUPT SERVICE ROUTINE -------------------- */

// encoder interrupt service routine output
volatile unsigned long encoder_ticks = 0;
void encoder_isr() { encoder_ticks++; }

// 8-bit processor, so reading encoder_ticks will take multiple steps
// prevents interrupt from modifying encoder_ticks while we are reading it
inline unsigned long get_tick_count() {
  unsigned long ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ticks = encoder_ticks;
  }
  return ticks;
}

/* ------------------ MOTOR CONTROL FUNCTIONS ------------------------------ */

inline void motor_fwd() { digitalWrite(MOTOR_INA1, HIGH); digitalWrite(MOTOR_INA2, LOW);  }
inline void motor_rev() { digitalWrite(MOTOR_INA1, LOW);  digitalWrite(MOTOR_INA2, HIGH); }
inline void motor_cst() { digitalWrite(MOTOR_INA1, HIGH); digitalWrite(MOTOR_INA2, HIGH); }

// attempt to match actual velocity to this function
inline float get_velocity_reference(float mm_travelled) {
  float mm_remaining = mm_target_distance - mm_travelled;
  return min(max_vel, sqrt(2 * slowdown_accel * mm_remaining)); // in mm/s
}

inline void update_motor_control(float dt_ms) {
  float dt = dt_ms / 1000.0; // convert dt from ms to seconds

  // 1. get the tick count
  unsigned long ticks = get_tick_count();

  // 2. measure actual velocity (mm/s)
  float delta_mm_travelled = (ticks - last_encoder_ticks) * mm_per_tick;
  last_encoder_ticks = ticks; // mark encoder ticks for next cycle
  float v_actual = delta_mm_travelled / dt;

  // 3. peg motor to 100% until either speedup phase ends or we hit max_vel
  if ((ticks < speedup_ticks) && (v_actual < max_vel)) {
    motor_fwd();
    digitalWrite(MOTOR_ENA, HIGH);
    return;
  }

#ifdef DEBUG_MODE
  if ((ticks < speedup_ticks) && (v_actual >= max_vel)) {
    write_log("max velocity ticks: \n");
    write_log_ulong(ticks);
    write_log("\n");
  }
#endif

  // 4. compute reference velocity from deceleration profile
  float v_ref = get_velocity_reference(ticks * mm_per_tick); // mm/s

  // 5. set motor power/direction using PID control
  float error = v_ref - v_actual;
  float motor_control = Kp * error;

  // 6. set motor direction  
  if (motor_control < 0) {
    motor_rev();
  } else {
    motor_fwd();
  }

  // 7. map motor control to 8 bit pwm value and send to motor
  analogWrite(MOTOR_ENA, (int)map(motor_control, -max_vel, max_vel, 0, 255));

  // 8.brake, detach encoder interrupt, and reset if we have reached the target
  if (ticks >= target_ticks) {
    motor_rev();
    digitalWrite(MOTOR_ENA, HIGH);
    delay(ms_brake_time);
    digitalWrite(MOTOR_ENA, LOW);
    motor_cst();

#ifdef DEBUG_MODE
    end_log();
#endif

    detachInterrupt(digitalPinToInterrupt(ENCODER_SIG));
    is_started = false;
  }
}

/* ------------------------------------------------------------------------- */

void setup() {
  pinMode(ENCODER_SIG, INPUT);
  pinMode(MOTOR_INA1, OUTPUT);
  pinMode(MOTOR_INA2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

#ifdef DEBUG_MODE
  Serial.begin(9600);
#endif
}

void loop() {
  // blink built-in led on each encoder tick
  digitalWrite(LED_BUILTIN, digitalRead(ENCODER_SIG));

  if (!is_started && (digitalRead(START_BTN) == HIGH)) {
    is_started         = true;
    encoder_ticks      = 0;
    last_encoder_ticks = 0;
    last_control_time  = millis();
    start_time         = last_control_time;
    attachInterrupt(digitalPinToInterrupt(ENCODER_SIG), encoder_isr, RISING);
  }

#ifdef DEBUG_MODE
  if (!is_started) {
    check_serial_command();
  }
#endif

  unsigned long now = millis();
  unsigned long dt_ms = now - last_control_time;
  if (is_started && dt_ms >= control_interval) {
    last_control_time = now;
    update_motor_control(dt_ms);
  }
}
