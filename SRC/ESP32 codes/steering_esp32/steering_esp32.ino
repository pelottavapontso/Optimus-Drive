#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber;
//std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2
#define CLOCKWISE 18
#define COUNTER_CLOCKWISE 19
#define ENCODER_A 32 // Pin for Encoder A
#define ENCODER_B 33 // Pin for Encoder B

volatile int encoder_value = 0; // Global variable for storing the encoder position

int zero = 10;

int P = 1;
//float I = 0.5;
//int D = 0;

int MAX_CYCLE = 240;
int MIN_CYCLE = 170;


int STEER = 1800; //Steering in tick

//volatile int error_int;
//volatile int last_error;

//volatile int prev_error;
// volatile int derivative = 0;
//volatile int last_time = 0;


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

void subscription_callback(const void * msgin)
{  
  //const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

   // Update last received message time
  lastMsgTime = millis();

  if (msg->data == 10.0) {
    encoder_value = 0;
    analogWrite(CLOCKWISE, 0);
    analogWrite(COUNTER_CLOCKWISE, 0);
  } else {

    int error = STEER*msg->data - encoder_value;

    //if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
        //error_int = 0;
    //}

//INT VARIABLES
    //error_int += error;
    //error_int = constrain(error_int, -STEER, STEER);
    

// DERIVATIVE VARIABLES

    //prev_error = error;
    // int current_time = millis();
    //int delta_time = current_time - last_time;
    //if (delta_time > 0) {
      //derivative = (error - last_error) / delta_time; 
    //}
    
//CONTROL CALCULATIONS
  
    //int PID = P*abs(error) + I*error_int + D*(error - prev_error);
    
    //int PID = P*abs(error) + I*abs(error_int);
    int PID = P*abs(error);

    //last_error = error;
    //last_time = current_time;
  
    
    int control = constrain(abs(PID), MIN_CYCLE, MAX_CYCLE);
    
    
  
 //PID control
     if (error >= zero) {
       analogWrite(CLOCKWISE, 0);
       analogWrite(COUNTER_CLOCKWISE, control);
     } else if (error <= -zero) {
       analogWrite(COUNTER_CLOCKWISE, 0);
       analogWrite(CLOCKWISE, control);
     } else {   
       analogWrite(CLOCKWISE, 0);
       analogWrite(COUNTER_CLOCKWISE, 0);
     }

  }
    
 }
  


void encoder_isr() {
  static int last_A = LOW;
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  
  if (A != last_A) { // Detect state change
    if (A == B) {
      encoder_value--;  // Counterclockwise
    } else {
      encoder_value++;  // Clockwise
    }
  }
  last_A = A;
}
  


void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  pinMode(CLOCKWISE, OUTPUT);
  pinMode(COUNTER_CLOCKWISE, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);
  
  // delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_steering"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));



}

void loop() {
  //delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));


  // Check if no message has been received for more than TIMEOUT ms
  if (millis() - lastMsgTime > TIMEOUT) {
    ESP.restart();
  }
  
}
