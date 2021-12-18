#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// The GPIO pins to which the sonar is wired
#define GPIO_ECHO 6
#define GPIO_TRIGGER 7

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer) {
    msg.data = read_range();
    fill_msg_stamp(msg.header.stamp);
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  } else {
    printf("Failed to publish range. Continuing.\n");
  }
}

/*void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
    msg.data = read_range()
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		
	}
}*/


/**
 * @brief Get the range value in meter.
 */
void read_range() {

  // Send an impulse trigger of 10us
  gpio_put(GPIO_TRIGGER, 1);
  sleep_us(10);
  gpio_put(GPIO_TRIGGER, 0);

  // Read how long is the echo
  uint32_t signaloff, signalon;
  do {
    signaloff = time_us_32();
  } while (gpio_get(GPIO_ECHO) == 0);

  do {
    signalon = time_us_32();
  } while (gpio_get(GPIO_ECHO) == 1);

  // Actual echo duration in us
  const float dt = signalon - signaloff;

  // distance in meter:
  // echo duration (us) x speed of sound (m/us) / 2 (round trip)
  return dt * 0.000343 / 2.0;
}

...

/**
 * @brief Read the range from the sensor,
 * fill up the ROS message and publish it.
 */



/*void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer) {
    range_msg.range = read_range();
    fill_msg_stamp(range_msg.header.stamp);
    RCSOFTCHECK(rcl_publish(&publisher, &range_msg, NULL));
  } else {
    printf("Failed to publish range. Continuing.\n");
  }
}*/



void appMain(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "freertos_int32_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"topic"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
