#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
//#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#include <driver/gpio.h>
//#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Macro functions
/*#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))*/
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


// Constants
//#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10

// PINS
#define LED_BUILTIN 2
/*#define PIN_LEFT_FORWARD 26
#define PIN_LEFT_BACKWARD 27
#define PIN_RIGHT_FORWARD 12
#define PIN_RIGHT_BACKWARD 13*/

// PWM Channels (Reserve channel 0 and 1 for camera)
/*#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5*/

// Other PWM settings
/*#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE*/

// These values are determined by experiment and are unique to every robot
/*#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MAX 4095   // Full speed (2^12 - 1)*/

//geometry_msgs__msg__Twist msg;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

void setupPins();
void setupRos();
//void cmd_vel_callback(const void *msgin);
//void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

    if((msg->data) == 0){
        gpio_set_level(LED_BUILTIN, 0);		
    }

    if((msg->data) == 1){
        gpio_set_level(LED_BUILTIN, 1);		
    }
	
}

void setupPins() {

    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);
    //gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
    
    
}

void setupRos() {
    // Micro ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "ros_esp32cam_diffdrive", "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "topic"));
/*
    // create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(FRAME_TIME),
        timer_callback));*/

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    //RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME * 1000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

// Main
void appMain(void *arg) {
    setupPins();
    setupRos();
    usleep(SLEEP_TIME * 1000);
}
