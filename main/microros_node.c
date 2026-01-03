#include "microros_node.h"

// micro_ros dependencies
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include "std_msgs/msg/float64_multi_array.h"

// esp-idf dependcies
#include "esp_log.h"
#include "esp_timer.h"

#include <rmw_microros/rmw_microros.h> //ros-esp-dds middleware
#include <uros_network_interfaces.h>   //pura transport sambhal lega ye

static const char *TAG = "micro-ros";

// command storage
static double left_cmd = 0.0;
static double right_cmd = 0.0;
static int64_t last_cmd_time_ms = 0;

// micro-ros object
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t cmd_sub;
static std_msgs__msg__Float64MultiArray cmd_msg; //memory storage for executor : agent se jo bhi data aayega executor isme dalega , phir callback ko isse box /ka address de dega 

// time conversion: qki esp nano second mai kaam karta

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static void wheel_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;

    if (msg->data.size < 2)
    {
        ESP_LOGW(TAG, "Invalid command message, size: %ld", (long)msg->data.size);
        return;
    }

    left_cmd = msg->data.data[0];
    right_cmd = msg->data.data[1];
    last_cmd_time_ms = now_ms();

    ESP_LOGI(TAG, "Received command - left: %.3f, right: %.3f", left_cmd, right_cmd);
}
void microros_init(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_ret_t ret;

    // Configure agent IP and port
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init options: %ld", (long)ret);
        return;
    }
    
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    ret = rmw_uros_options_set_udp_address("192.168.21.70", "8888", rmw_options);
    if (ret != RMW_RET_OK) {
        ESP_LOGE(TAG, "Failed to set UDP address: %ld", (long)ret);
        return;
    }
    ESP_LOGI(TAG, "Connecting to agent at 192.168.21.70:8888");
    
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init support: %ld", (long)ret);
        return;
    }
    ESP_LOGI(TAG, "Support initialized successfully");

    ret = rclc_node_init_default(&node, "esp32_node", "", &support);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init node: %ld", (long)ret);
        return;
    }
    ESP_LOGI(TAG, "Node created: esp32_node");

    ret = rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/wheel_commands");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create subscriber: %ld", (long)ret);
        return;
    }
    ESP_LOGI(TAG, "Subscriber created on /wheel_commands");

    // Initialize message memory BEFORE adding to executor
    std_msgs__msg__Float64MultiArray__init(&cmd_msg);
    cmd_msg.data.capacity = 10;  // Allocate space for up to 10 values
    cmd_msg.data.size = 0;
    cmd_msg.data.data = (double*)allocator.allocate(cmd_msg.data.capacity * sizeof(double), allocator.state);
    if (cmd_msg.data.data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for message data");
        return;
    }
    ESP_LOGI(TAG, "Message buffer allocated (capacity: %ld)", (long)cmd_msg.data.capacity);

    //executor
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init executor: %ld", (long)ret);
        return;
    }
    
    ret = rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &wheel_cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add subscription to executor: %ld", (long)ret);
        return;
    }

    ESP_LOGI(TAG, "micro-ROS node initialized successfully!");
}
void microros_spin(void)
{
    rclc_executor_spin_some(&executor,RCL_MS_TO_NS(5));
}