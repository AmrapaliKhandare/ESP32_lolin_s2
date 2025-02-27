// Include necessary libraries
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>  // Required for WiFi connectivity

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#error This example is only available for Arduino framework with WiFi transport.
#endif

// Define ROS2 objects
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Define WiFi Credentials
char ssid[] = "NYU_1143_#2";       
char password[] = "Chennai_Boizz";    

// Define micro-ROS agent IP and port
IPAddress agent_ip(192, 168, 1, 117);
size_t agent_port = 8888;

float prev_linear_x = 0;
float prev_angular_z = 0;

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function to enter an infinite loop if an error occurs
void error_loop() {
  while (1) {
    delay(100);
  }
}

// Timer callback function
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (Serial1.available()) {
      digitalWrite(15, HIGH);
      String data = Serial1.readStringUntil('\n'); // Read full line
      float bot_x, bot_y, bot_ang, ping_ang, ping_dist, vol;
      if (sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f", &bot_x, &bot_y, &bot_ang, &ping_ang, &ping_dist, &vol) == 6) {
        msg.data.data[0] = bot_x;
        msg.data.data[1] = bot_y;
        msg.data.data[2] = bot_ang;
        msg.data.data[3] = ping_ang;
        msg.data.data[4] = ping_dist;
        msg.data.data[5] = vol;
      } else {
        for (int i = 0; i < 6; i++) msg.data.data[i] = 0;
      }
    } else {
      for (int i = 0; i < 6; i++) msg.data.data[i] = 0;
    }
    rcl_publish(&publisher, &msg, NULL);
  }
}

// Callback function for /cmd_vel subscriber
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  // if(linear_x > prev_linear_x){
  //   Serial1.println("W");  // Forward
  // } else if (linear_x < prev_linear_x){
  //   Serial1.println("X");  // Backward
  // } else if (angular_z > prev_angular_z){
  //   Serial1.println("A");  // Left turn
  // } else if (angular_z < prev_angular_z){
  //   Serial1.println("D");  // Left turn
  // } else {
  //   Serial1.println("S"); 
  // }
  // prev_linear_x = linear_x;
  // prev_angular_z = angular_z;
  if (linear_x > 0) {
    Serial1.println("W");  // Forward
  } else if (linear_x < 0) {
    Serial1.println("X");  // Backward
  } else if (angular_z > 0) {
    Serial1.println("A");  // Left turn
  } else if (angular_z < 0) {
    Serial1.println("D");  // Right turn
  } else {
    Serial1.println("S");
  }
}

void setup() {
  // Start Serial (for debugging)
  delay(1000);

  pinMode(15, OUTPUT);
  // Initialize UART (for Lolin S2 use GPIO 37 as RX, GPIO 39 as TX)
  Serial1.begin(9600, SERIAL_8N1, 37, 39);  // RX=GPIO37, TX=GPIO39

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Set up micro-ROS communication over WiFi
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

  // Get the default allocator
  allocator = rcl_get_default_allocator();

  // Initialize ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize ROS node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_wifi_node", "", &support));

  // Initialize publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_wifi_publisher"));

  // Initialize subscriber for /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Set up a timer to publish messages every 1 second
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  // Initialize message data
  msg.data.size = 6;
  msg.data.capacity = 6;
  msg.data.data = (float_t*)malloc(6 * sizeof(float_t));
}

void loop() {
  // Run executor to process ROS tasks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // delay(10);
}
