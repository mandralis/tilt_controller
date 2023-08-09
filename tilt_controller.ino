#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ezButton.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// main executor
rclc_executor_t executor;

// publisher
rcl_publisher_t publisher;
std_msgs__msg__String msg_debug;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// subscriber state
rcl_subscription_t subscriber_state;
std_msgs__msg__Int32 msg_state;
rclc_executor_t executor_sub_state;

// subscriber desired angle
rcl_subscription_t subscriber_angle;
std_msgs__msg__Float32 msg_angle;
rclc_executor_t executor_sub_angle;

float LOOP_PERIOD_MS = 100.0;

#define LED_PIN 2
#define ENCA 13
#define ENCB 12
#define PWM 14
#define DIR 27
#define LIM_DOWN 26
#define LIM_UP 25

ezButton limDown(LIM_DOWN);
ezButton limUp(LIM_UP);

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }



// limit counter
float limitCounter = 0.0;
float STALL_SECONDS = 0.5;

// arm/disarm
bool armed = false;
bool motor_running = false;
bool tracking = false;

// motor control
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile int posi = 0;
float eprev = 0;
float eintegral = 0;
float kp = 2.00;
float kd = 0;
float ki = 0;

// tilt angle
float angle = 0.0;
float desired_angle = 0.0;

int N(float phi) {
  // calibration table for angle in degrees vs number of encoder counts
  const int numValues = 29;
  double phiValues[29] = { 0, 0.9, 2.2, 3.7, 5.5, 7.6, 10.0, 12.7, 15.3, 18.4, 21.7, 25.7, 28.8, 32.5, 36.3, 40.3, 44.3, 48.3, 52.3, 56.2, 60.3, 63.4, 67.4, 71.3, 75.1, 78.8, 82.7, 85.3, 88.1 };
  double NValues[29] = { 0, 50, 101, 151, 202, 253, 305, 357, 407, 458, 509, 560, 601, 652, 703, 754, 804, 855, 905, 957, 1009, 1059, 1108, 1158, 1207, 1254, 1303, 1352, 1375 };
  return -map(phi, 0, 90, 0, 683);
}

void setMotor(int dir, int pwmVal, int pwm, int dir_pin) {
  motor_running = true;
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(dir_pin, LOW);
  } else if (dir == -1) {
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }
}

void stopMotor(int pwmVal) {
  motor_running = false;
  tracking = false;
  analogWrite(pwmVal, 0.0);
}

void arm() {
  tracking = false;
  armed = true;
}

void disarm() {
  armed = false;
  tracking = false;
}

void reset_encoder(bool down) {
  (down) ? posi = 0 : posi = 683;
}

void goDrive() {
  tracking = false;
  setMotor(-1, 100, PWM, DIR);
}

void goFly() {
  tracking = false;
  setMotor(1, 255, PWM, DIR);
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

void track() {
  // get target angle
  int target = N(desired_angle);

  // read encoder position
  int pos = 0;
  // portENTER_CRITICAL(&mux);
  pos = posi;
  // portEXIT_CRITICAL(&mux);

  // char *out = (char*)malloc(13 * sizeof(char));
  // sprintf(out, "posi: %d; target: %d", posi, target);
  // msg_debug.data.data = out;

  char *out = (char *)malloc(13 * sizeof(char));
  sprintf(out, "desired_angle: %d", int(desired_angle));
  msg_debug.data.data = out;
  RCSOFTCHECK(rcl_publish(&publisher, &msg_debug, NULL));

  // PID
  int e = pos - target;
  float dedt = (e - eprev) / LOOP_PERIOD_MS;
  eintegral = eintegral + e * LOOP_PERIOD_MS;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, DIR);

  // store previous error
  eprev = e;
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(LOOP_PERIOD_MS);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // RCSOFTCHECK(rcl_publish(&publisher, &msg_debug, NULL));

    limDown.loop();
    limUp.loop();
    int stateLimDown = limDown.getState();
    int stateLimUp = limUp.getState();

    // if (stateLimDown == 0) {
    //   msg_debug.data.data = "down: 0";
    // } else{
    //   msg_debug.data.data = "down: 1";
    // }

    // if (stateLimUp == 0) {
    //   msg_debug.data.data = "up: 0";
    // } else{
    //   msg_debug.data.data = "up: 1";
    // }

    // (motor_running) ? msg_debug.data.data = "motor running !" : msg_debug.data.data = "motor stopped !";

    // time difference
    if ((stateLimDown == 0 or stateLimUp == 0) and motor_running) {
      limitCounter += LOOP_PERIOD_MS / 1e3;
      if (limitCounter > STALL_SECONDS) {
        stopMotor(PWM);
        limitCounter = 0.0;
        disarm();
        msg_debug.data.data = "motor disarmed and encoder reset!";
        if (stateLimDown == 0) {
          bool down = true;
          reset_encoder(down);
          angle = 0.0;
        }
        if (stateLimUp == 0) {
          bool down = false;
          reset_encoder(down);
          angle = PI / 2;
        }
      }
    }

    //TRACKING
    if (tracking) {track();}

  }
}

void subscription_callback_state(const void *msgin) {
  const std_msgs__msg__Int32 *msg_state = (const std_msgs__msg__Int32 *)msgin;

  switch (msg_state->data) {
    case 0:  // arm
      arm();
      msg_debug.data.data = "motor armed!";
      RCSOFTCHECK(rcl_publish(&publisher, &msg_debug, NULL));
      break;
    case 1:  // drive
      if (armed) {
        goDrive();
        msg_debug.data.data = "going drive!";
        RCSOFTCHECK(rcl_publish(&publisher, &msg_debug, NULL));
      } else {
        msg_debug.data.data = "[drive] motor is disarmed - cannot execute action!";
      }
      break;
    case 2:  // fly
      if (armed) {
        goFly();
      } else {
        msg_debug.data.data = "[fly] motor is disarmed - cannot execute action!";
      }
      break;
    case 3:  // track
      if (armed) {
        tracking = true;
      } else {
        msg_debug.data.data = "[track] motor is disarmed - cannot execute action!";
      }
      break;
    case 4:  // stop
      stopMotor(PWM);
      disarm();
      msg_debug.data.data = "motor stopped and disarmed !";
      break;
    default:
      stopMotor(PWM);
      disarm();
      msg_debug.data.data = "motor stopped and disarmed !";
      break;
  }
}

void subscription_callback_angle(const void *msgin) {
  const std_msgs__msg__Float32 *msg_angle = (const std_msgs__msg__Float32 *)msgin;
  desired_angle = msg_angle->data;

  // printf("Received: %d\n", msg_angle->data);

  // char *out = (char *)malloc(13 * sizeof(char));
  // sprintf(out, "desired_angle: %f", desired_angle);
  // msg_debug.data.data = out;

  // RCSOFTCHECK(rcl_publish(&publisher, &msg_debug, NULL));

}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  limDown.setDebounceTime(50);
  limUp.setDebounceTime(50);

  digitalWrite(LED_PIN, HIGH);
  
  digitalWrite(PWM, LOW);
  digitalWrite(DIR, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_xiao_node", "", &support));

  // create subscriber_state
  RCCHECK(rclc_subscription_init_default(
    &subscriber_state,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_state"));

  // create subscriber_state
  RCCHECK(rclc_subscription_init_default(
    &subscriber_angle,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_angle"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "micro_debug"));

  // create timer, called every 100 ms to publish heartbeat
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor 
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_state, &msg_state, &subscription_callback_state, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_angle, &msg_angle, &subscription_callback_angle, ON_NEW_DATA));

  // // create executor
  // RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  // RCCHECK(rclc_executor_init(&executor_sub_state, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_subscription(&executor_sub_state, &subscriber_state, &msg_state, &subscription_callback_state, ON_NEW_DATA));

  // RCCHECK(rclc_executor_init(&executor_sub_angle, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_subscription(&executor_sub_angle, &subscriber_angle, &msg_angle, &subscription_callback_angle, ON_NEW_DATA));
}

void loop() {
  delay(LOOP_PERIOD_MS);
  RCCHECK(rclc_executor_spin_some(&executor,  RCL_MS_TO_NS(LOOP_PERIOD_MS)));
  // RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(LOOP_PERIOD_MS)));
  // RCCHECK(rclc_executor_spin_some(&executor_sub_state, RCL_MS_TO_NS(LOOP_PERIOD_MS)));
  // RCCHECK(rclc_executor_spin_some(&executor_sub_angle, RCL_MS_TO_NS(LOOP_PERIOD_MS)));
}
