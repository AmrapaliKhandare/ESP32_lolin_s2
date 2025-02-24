// Include necessary libraries
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>  // Required for WiFi connectivity

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#error This example is only available for Arduino framework with WiFi transport.
#endif

// Define ROS2 objects for a publisher, a message, an executor, support objects, an allocator, a node, and a timer
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Define WiFi Credentials (as mutable char arrays)
char ssid[] = "1057-SRRR";       // Your WiFi SSID (MUST be mutable)
char password[] = "1057SRRR";    // Your WiFi password (MUST be mutable)

// Define micro-ROS agent IP and port
IPAddress agent_ip(192, 168, 1, 114);
size_t agent_port = 8888;

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
    // Publish the message
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // Increment the message data
    msg.data += 2;
  }
}

void setup() {
  // Start Serial (for debugging)
  Serial.begin(115200);
  delay(1000);

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // Set up micro-ROS communication over WiFi
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  Serial.println("Connected to micro-ROS agent via WiFi");

  // Allow some time for everything to initialize
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_wifi_publisher"));

  // Set up a timer to publish messages every 1 second
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize executor and add timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize message data
  msg.data = 0;
}

void loop() {
  // Run executor to process ROS tasks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(100);
}
