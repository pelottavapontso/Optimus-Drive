#include <Arduino.h> 
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

#define PUSH_1 18
#define PULL_1 19
#define POT_1 33

int pos_1;  // Defining int value for gas feedback

#define PUSH_2 22
#define PULL_2 23
#define POT_2 32

int pos_2;   // Defining int value for brake feedback
int zero = 3; // Error tolerance for pedals

// PID controller gains initialization
int P = 10;

int MAX_CYCLE = 240;    // Maximum PWM-control command 
int MIN_CYCLE = 240;    // Minimum -||-

volatile int GAS_START = 0;    // In BIT
volatile int BRAKE_START = 0;  // In BIT

volatile int hand_brake = 0;
float HANDBRAKE_POS = 1.75;    // In the range of 0-2 of the movement

volatile int start_pos = 0;   // BIT
volatile int error;

// Watchdog timer implementation
unsigned long lastMsgTime = 0;  // Last received message time
const unsigned long TIMEOUT = 5000;  // 5 seconds timeout

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  for (int i = 0; i < 10; i++) {  // Blink LED for 1 second, then reset
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  ESP.restart();
}

void control_callback(float command, int PUSH, int PULL, int pos, int pedal) {
  if (pedal == 1) {
    start_pos = GAS_START;
  } else {
    start_pos = BRAKE_START;
  }

  error = start_pos * (100.0 / 3950.0) + abs(command) * 50.0 * (100-0 - start_pos * (100.0 / 3950.0))/100.0 - pos * (100.0 / 3950.0);

  int PID = P * error;
  int control = constrain(abs(PID), MIN_CYCLE, MAX_CYCLE);
  
  if (error >= zero) {
    analogWrite(PULL, 0);
    analogWrite(PUSH, control);
  } else if (error <= -zero) { 
    analogWrite(PUSH, 0);
    analogWrite(PULL, control);
  } else {
    analogWrite(PUSH, 0);
    analogWrite(PULL, 0);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  // Update last received message time
  lastMsgTime = millis();

  if (msg->data < -2.0 || msg->data > 2.0) {
    if (msg->data == 10.0) {
      hand_brake = 1;
    } else if (msg->data == -10.0) {
      hand_brake = 0;
    } else if (msg->data == 20.0) {
      GAS_START = pos_1;
    } else if (msg->data == -20.0) {
      BRAKE_START = pos_2;
    } else if (msg->data == 30.0) {
      GAS_START = 0;
      BRAKE_START = 0;
    }
  }

  if (hand_brake == 1) {
    control_callback(HANDBRAKE_POS, PUSH_2, PULL_2, pos_2, 2);
  } else {
    // GAS
    if (msg->data > 0 && msg->data <= 2.0) {
      control_callback(msg->data, PUSH_1, PULL_1, pos_1, 1);
    } else {
      control_callback(0, PUSH_1, PULL_1, pos_1, 1);
    }

    // BRAKE
    if (msg->data < 0 && msg->data >= -2.0) {
      control_callback(msg->data, PUSH_2, PULL_2, pos_2, 2);
    } else {
      control_callback(0, PUSH_2, PULL_2, pos_2, 2);
    }
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  pinMode(PUSH_1, OUTPUT);
  pinMode(PULL_1, OUTPUT);

  pinMode(PUSH_2, OUTPUT);
  pinMode(PULL_2, OUTPUT);
  
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_pedal"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Initialize last message time
  lastMsgTime = millis();
}

void loop() {
  pos_1 = analogRead(POT_1);
  pos_2 = analogRead(POT_2);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Check if no message has been received for more than TIMEOUT ms
  if (millis() - lastMsgTime > TIMEOUT) {
    ESP.restart();
  }
}
