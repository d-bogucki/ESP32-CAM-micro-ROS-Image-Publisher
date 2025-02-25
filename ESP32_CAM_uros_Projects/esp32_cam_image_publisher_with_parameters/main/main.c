//===========================================================================================================================================================================
// This application creates a micro-ROS node that publishes a compressed image (jpeg) message over the ROS2 Dataspace
//===========================================================================================================================================================================

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Standard Library Headers
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// FreeRTOS and ESP Headers
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_timer.h"
#include "driver/gpio.h"
/* Note on the LOGS:
Inside the code all the "DEBUG" level logs (ESP_LOGD) are not enabled by default
For enabling them simply change the sdkconfig file via "idf.py menuconfig" (without having to modify the code) using a ESP-IDF terminal like below:
-) Component config --> Log output --> Default Log Verbosity --> Debug
-) save and exit
-) build project and flash again
*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Camera driver
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "esp_camera.h"
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// micro-ROS Headers
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
// Header for Image ROS msg type:
#include <sensor_msgs/msg/compressed_image.h> 
// Headers for the parameters:
#include <rclc_parameter/rclc_parameter.h>

// Macros functions for errors:
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// PIN MAP definition
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define FLASH_GPIO 4  		// GPIO for the flash LED
#define CAM_PIN_PWDN 32  	// GPIO pin for camera power down line
#define CAM_PIN_RESET -1 	// software reset will be performed 
#define CAM_PIN_XCLK 0   	// GPIO pin for camera XCLK line
#define CAM_PIN_SIOD 26  	// GPIO pin for camera SDA line
#define CAM_PIN_SIOC 27  	// GPIO pin for camera SCL line
#define CAM_PIN_D7 35   	// GPIO pin for camera D7 line
#define CAM_PIN_D6 34   	// GPIO pin for camera D6 line
#define CAM_PIN_D5 39   	// GPIO pin for camera D5 line
#define CAM_PIN_D4 36   	// GPIO pin for camera D4 line
#define CAM_PIN_D3 21   	// GPIO pin for camera D3 line
#define CAM_PIN_D2 19   	// GPIO pin for camera D2 line
#define CAM_PIN_D1 18   	// GPIO pin for camera D1 line
#define CAM_PIN_D0 5    	// GPIO pin for camera D0 line
#define CAM_PIN_VSYNC 25 	// GPIO pin for camera VSYNC line   
#define CAM_PIN_HREF 23  	// GPIO pin for camera HREF line
#define CAM_PIN_PCLK 22  	// GPIO pin for camera PCLK line

// Module TAG definition: (for esp logs)
static const char *TAG = "micro_ROS_image_publisher";

// Default JPEG quality:
#define JPEG_QUALITY_DEFAULT 20

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Camera Chip Configuration 
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#if ESP_CAMERA_SUPPORTED    

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
	// Digital Video Port parallel 8-bit image data output:
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
	// Frame, Line and Pixel timings:
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    // Settings:
    .xclk_freq_hz = 20000000, 		// 20 MHz  of master clock
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,    
    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG    
	.frame_size = FRAMESIZE_CIF,    // QQVGA-UXGA Do not use sizes above QVGA when not JPEG
	// Other frame sizes:
	// FRAMESIZE_QVGA --> 320x240
	// FRAMESIZE_CIF  --> 400x296
	// FRAMESIZE_VGA  --> 640x480
	// FRAMESIZE_SVGA --> 800x600
	// FRAMESIZE_HD   --> 1280x720
	// FRAMESIZE UXGA --> 1600x1200 (2 MP maximum resolution)
	.jpeg_quality = JPEG_QUALITY_DEFAULT, 	// 0-63 lower number means higher quality
    .fb_count = 2,       					// if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,  	// frame buffer will be in the external PSRAM 
	.grab_mode = CAMERA_GRAB_LATEST, 		// other mode: CAMERA_GRAB_WHEN_EMPTY
};

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Camera Chip Initialization Function:
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

static esp_err_t init_camera()
{
    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

#endif

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Global variables declaration for micro-ROS publisher and Image message
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Publisher Object:
rcl_publisher_t publisher;

sensor_msgs__msg__CompressedImage msg_static;	// C99 struct representantion of the .msg ROS structure

static int64_t last_callback = 0;				// time buffer for time logs

// Parameter server object:
rclc_parameter_server_t param_server;				

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters Callback Function:
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void) context;

  // Pointer to the current image sensor control structure:	
  sensor_t *s = esp_camera_sensor_get();

  if (old_param == NULL && new_param == NULL) {
    printf("Callback error, both parameters are NULL\n");
    return false;
  }

  if (old_param == NULL) {
    printf("Creating new parameter %s\n", new_param->name.data);
  } else if (new_param == NULL) {
    printf("Deleting parameter %s\n", old_param->name.data);
  } else {
    printf("Parameter %s modified.", old_param->name.data);
    switch (old_param->value.type) {
      case RCLC_PARAMETER_BOOL:
        printf(
          " Old value: %d, New value: %d (bool)", old_param->value.bool_value,
          new_param->value.bool_value);
        break;
      case RCLC_PARAMETER_INT:
        printf(
          " Old value: %lld, New value: %lld (int)", old_param->value.integer_value,
          new_param->value.integer_value);
		  if (strcmp(old_param->name.data, "jpeg_quality") == 0) {
			s->set_quality(s, new_param->value.integer_value);			// jpeg quality change
		  }
		  if (strcmp(old_param->name.data, "frame_size") == 0) {
			s->set_framesize(s, new_param->value.integer_value);		// frame size change
		  }
		  if (strcmp(old_param->name.data, "brightness") == 0) {
			s->set_brightness(s, new_param->value.integer_value);		// brightness change
		  }
		  if (strcmp(old_param->name.data, "contrast") == 0) {
			s->set_contrast(s, new_param->value.integer_value);			// contrast change
		  }
		  if (strcmp(old_param->name.data, "saturation") == 0) {
			s->set_saturation(s, new_param->value.integer_value);		// saturation change
		  }
		  if (strcmp(old_param->name.data, "sharpness") == 0) {
			s->set_sharpness(s, new_param->value.integer_value);		// sharpness change
		  }
		  if (strcmp(old_param->name.data, "special_effect") == 0) {
			s->set_special_effect(s, new_param->value.integer_value);	// special effect change
		  }
		  if (strcmp(old_param->name.data, "flash_led") == 0) {
			gpio_set_level(GPIO_NUM_4, new_param->value.integer_value); // flash LED ON/OFF
		  }
		  if (strcmp(old_param->name.data, "ae_level") == 0) {
			s->set_ae_level(s, new_param->value.integer_value);         // auto-exposure level change
		  }
		  if (strcmp(old_param->name.data, "h_mirror") == 0) {
			s->set_hmirror(s, new_param->value.integer_value);   		// horizontal mirror
		  }
		  if (strcmp(old_param->name.data, "v_mirror") == 0) {
			s->set_vflip(s, new_param->value.integer_value);   			// vertical mirror
		  }
		  if (strcmp(old_param->name.data, "colorbar") == 0) {
			s->set_colorbar(s, new_param->value.integer_value);   		// colorbar for color testing
		  }
		  if (strcmp(old_param->name.data, "wb_mode") == 0) {
			s->set_wb_mode(s, new_param->value.integer_value);   		// white-balance mode change
		  }
		  if (strcmp(old_param->name.data, "awb") == 0) {
			s->set_whitebal(s, new_param->value.integer_value);   		// enable/disable white balance
		  }
		  if (strcmp(old_param->name.data, "aec") == 0) {
			s->set_exposure_ctrl(s, new_param->value.integer_value);   	// enable/disable automatic exposure control
		  }
		  if (strcmp(old_param->name.data, "aec_2") == 0) {
			s->set_aec2(s, new_param->value.integer_value);   		    // enable/disable automatic exposure control 2
		  }
		  if (strcmp(old_param->name.data, "aec_value") == 0) {
			s->set_aec_value(s, new_param->value.integer_value);   		// auto exposure control value change
		  }
		  if (strcmp(old_param->name.data, "agc") == 0) {
			s->set_gain_ctrl(s, new_param->value.integer_value);   		// enable/disable automatic gain control
		  }
		  if (strcmp(old_param->name.data, "agc_gain") == 0) {
			s->set_agc_gain(s, new_param->value.integer_value);   		// agc gain change
		  }
		  if (strcmp(old_param->name.data, "gain_ceiling") == 0) {
			s->set_gainceiling(s, new_param->value.integer_value);   	// change gain ceiling
		  }
		  if (strcmp(old_param->name.data, "bpc") == 0) {
			s->set_bpc(s, new_param->value.integer_value);   			// enable/disable black pixel correction
		  }
		  if (strcmp(old_param->name.data, "wpc") == 0) {
			s->set_wpc(s, new_param->value.integer_value);   			// enable/disable white pixel correction
		  }
		  if (strcmp(old_param->name.data, "raw_gma") == 0) {
			s->set_raw_gma(s, new_param->value.integer_value);   		// enable/disable gamma correction
		  }
		  if (strcmp(old_param->name.data, "lenc") == 0) {
			s->set_lenc(s, new_param->value.integer_value);   			// enable/disable lens correction
		  }
		  if (strcmp(old_param->name.data, "denoise") == 0) {
			s->set_denoise(s, new_param->value.integer_value);   		// enable/disable denoise
		  }
        break;
      case RCLC_PARAMETER_DOUBLE:
        printf(
          " Old value: %f, New value: %f (double)", old_param->value.double_value,
          new_param->value.double_value);
        break;
      default:
        break;
    }
    printf("\n");
  }
  return true;
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Timer Callback function Definition
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{	

// Time variables for logs in debug mode:	
int64_t time_0 = esp_timer_get_time();	// [us]
int64_t elapsed_time, time_pub_1, time_pub_2, elapsed_time_publishing;

	
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {

		// Time Log:
		elapsed_time = (time_0 - last_callback)/1000; 	// [ms]
		ESP_LOGD(TAG, "Callback Init! Time elapsed between the last callback start : %lld [ms]", elapsed_time);
		last_callback = time_0;		
		
		// Getting the pointer to a frame buffer:
		camera_fb_t *pic = esp_camera_fb_get();			// use pic->buf to access the image	

		if (pic != NULL) {

			// Logging the msg_static capacity:
			ESP_LOGD(TAG, "the msg_static capacity is : %d", msg_static.data.capacity);  	
			if (pic->len <= msg_static.data.capacity) {
				
				// Setting the valid number of elements of the image buffer:
				msg_static.data.size = pic->len;

				// Copying the image buffer of the camera driver into the data field of the micro-ROS message:
				memcpy(msg_static.data.data, pic->buf, pic->len); 

				// Logging Image size and location:
				ESP_LOGD(TAG, "Image captured! Size: %zu [bytes], Resolution: (%d x %d), Camera Image buffer address: %p, micro-ROS image buffer address: %p",  
					pic->len, pic->width, pic->height, pic->buf, msg_static.data.data);

				// Setting other msg_static fields, header and format:
				msg_static.header.frame_id = micro_ros_string_utilities_set(msg_static.header.frame_id, "myframe");
				msg_static.format = micro_ros_string_utilities_set(msg_static.format, "jpeg");
				
				// Message Publishing:
				time_pub_1 = esp_timer_get_time();	    
                RCSOFTCHECK(rcl_publish(&publisher, &msg_static, NULL));
                time_pub_2 = esp_timer_get_time();

                elapsed_time_publishing = (time_pub_2 - time_pub_1)/1000; 	// [ms]
                ESP_LOGD(TAG, "The rcl_publish function took %lld [ms]\n", elapsed_time_publishing);
			}
			else
			{
				ESP_LOGE(TAG, "Image size exceeeds micro-ROS msg capacity, image size was: %zu [bytes], capacity is: %zu [bytes]", 
					pic->len, msg_static.data.capacity); 
			}

			// Deallocating the image buffer:	
			esp_camera_fb_return(pic);
		}
		else
		{
			ESP_LOGE(TAG, "Image not captured correctly! esp_camera_fb_get() function returned a NULL pointer");
		} 
	}	
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// micro-ROS task function
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void micro_ros_task(void * arg)
{   

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// CREATE A NODE WITH CUSTOM OPTIONS:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Initialize micro-ROS allocator:
    rcl_allocator_t allocator = rcl_get_default_allocator();
    // Initialize and modify "rcl" options:
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

    // RUN-TIME MIDDLEWARE CONFIGURATION:
	#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Wi-Fi UDP Setup:
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	#endif
	// Initialize rclc support object with custom options:
    rclc_support_t support;
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node object:
	rcl_node_t node;
	const char * node_name = "esp32_image_publisher";
	RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// CREATE PARAMETER SERVICE:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	// Initialization with custom options:
	const rclc_parameter_options_t param_options = {
    .notify_changed_over_dds = true,
    .max_params = 25,
    .allow_undeclared_parameters = false,
    .low_mem_mode = false 
	};	

	// Initialize parameter server with configured options:
	RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_options));

	// Add Parameters:
	RCCHECK(rclc_add_parameter(&param_server, "jpeg_quality", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "frame_size", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "brightness", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "contrast", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "saturation", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "sharpness", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "special_effect", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "flash_led", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "h_mirror", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "v_mirror", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "colorbar", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "wb_mode", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "awb", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "aec", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "aec_2", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "ae_level", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "aec_value", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "agc", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "agc_gain", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "gain_ceiling", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "bpc", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "wpc", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "raw_gma", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "lenc", RCLC_PARAMETER_INT));
	RCCHECK(rclc_add_parameter(&param_server, "denoise", RCLC_PARAMETER_INT));


	// Set parameters value:
    RCCHECK(rclc_parameter_set_int(&param_server, "jpeg_quality", JPEG_QUALITY_DEFAULT));
	RCCHECK(rclc_parameter_set_int(&param_server, "frame_size", FRAMESIZE_CIF));
    RCCHECK(rclc_parameter_set_int(&param_server, "brightness", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "contrast", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "saturation", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "sharpness", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "special_effect", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "flash_led", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "h_mirror", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "v_mirror", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "colorbar", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "wb_mode", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "awb", 1));
	RCCHECK(rclc_parameter_set_int(&param_server, "aec", 1));
	RCCHECK(rclc_parameter_set_int(&param_server, "aec_2", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "ae_level", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "aec_value", 335));
	RCCHECK(rclc_parameter_set_int(&param_server, "agc", 1));
	RCCHECK(rclc_parameter_set_int(&param_server, "agc_gain", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "gain_ceiling", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "bpc", 0));
	RCCHECK(rclc_parameter_set_int(&param_server, "wpc", 1));
	RCCHECK(rclc_parameter_set_int(&param_server, "raw_gma", 1));
	RCCHECK(rclc_parameter_set_int(&param_server, "lenc", 1));
	RCCHECK(rclc_parameter_set_int(&param_server, "denoise", 0));


	// Parameters Constraints:
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "jpeg_quality", 0, 63, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "frame_size", 0, 10, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "brightness", -2, 2, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "contrast", -2, 2, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "saturation", -2, 2, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "sharpness", -2, 2, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "special_effect", 0, 6, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "flash_led", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "h_mirror", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "v_mirror", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "colorbar", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "wb_mode", 0, 4, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "awb", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "aec", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "aec_2", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "ae_level", -2, 2, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "aec_value", 0, 1200, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "agc", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "agc_gain", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "gain_ceiling", 0, 6, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "bpc", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "wpc", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "raw_gma", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "lenc", 0, 1, 1));
	RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "denoise", 0, 1, 1));
	

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// CREATING A PUBLISHER:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Topic name:
	const char * topic_name = "image/compressed";

	// Default --> "reliable", use "rclc_publisher_init_best_effort()" for best-effort:
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher,
		&node,
    	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
		topic_name)); 	


	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// CREATING A TIMER:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Timer period in nanoseconds: 
	const unsigned int timer_period = RCL_MS_TO_NS(15);
	
	// Create and initialize timer object:
	rcl_timer_t timer;

	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		timer_period,
		timer_callback));	

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// CREATE AND INITIALIZE THE EXECUTOR:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Create executor:
	rclc_executor_t executor;
	//RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1, &allocator));
	// During the initialization the rclc executor dynamically allocates memory for the total number of "handles",
	// hence the number of subscription, timers, clients, services and so on.

	// Adding parameter server to the executor:
	RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));

    // Add timer to the executor:
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// MEMORY ALLOCATION FOR "CompressedImage" micro-ROS MESSAGE:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// micro-ROS utilities allows to configure the dynamic memory initialization using a micro_ros_utilities_memory_conf_t structure
	// If some member of this struct is set to zero, the library will use the default value.
	// Default values:
	// String -> 20 characters:
	// ROS 2 types sequences -> 5 elements
	// Basic types sequences -> 5 elements

	static micro_ros_utilities_memory_conf_t conf = {};

	// Custom capacity setting:
	conf.max_string_capacity = 30;
	conf.max_ros2_type_sequence_capacity = 5;
	conf.max_basic_type_sequence_capacity = 5;

	// Optionally this struct can store rules for specific members:
	micro_ros_utilities_memory_rule_t rules[] = {
		{"header.frame_id", 30},
		{"format",4},	// since "jpeg" is the longest acceptable format
		{"data", 13000} // Allocates up to 13000 bytes for the data field (compressed image data) (equal to the MTU size) 
	};
	
	conf.rules = rules;
	// Total number of rules defined in this configuration:
	conf.n_rules = sizeof(rules) / sizeof(rules[0]);
	
    // Allocates the dynamic memory required for a message:
	micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
		&msg_static,
		conf
	);

	// Getting the camera infos before the spin:
	sensor_t *s = esp_camera_sensor_get();
	ESP_LOGI(TAG, "Camera Default Settings:\n Brightness: %d, Contrast: %d, Saturation: %d, Sharpness: %d\n"
				   "Denoise: %d, Special effect: %d, WB mode: %d, AWB: %d, AWB Gain: %d, aec: %d, aec2: %d\n" 
				   "ae level: %d, aec value: %d, agc: %d, agc gain: %d, gainceiling: %d, bpc: %d, wpc: %d\n" 
				   "raw_gma: %d, lenc: %d, hmirror: %d, vflip: %d, dcw: %d, colorbar: %d\n", 
			 s->status.brightness, s->status.contrast, s->status.saturation, s->status.sharpness,
			 s->status.denoise, s->status.special_effect, s->status.wb_mode, s->status.awb, s->status.awb_gain, s->status.aec, s->status.aec2,
			 s->status.ae_level, s->status.aec_value, s->status.agc, s->status.agc_gain, s->status.gainceiling, s->status.bpc, s->status.wpc,
			 s->status.raw_gma, s->status.lenc, s->status.hmirror, s->status.vflip, s->status.dcw, s->status.colorbar);


    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// SPIN LOOP:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	}

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// CLEAN UP:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	RCCHECK(rclc_parameter_server_fini(&param_server, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// MAIN FUNCTION
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void app_main(void)
{

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

#if ESP_CAMERA_SUPPORTED
    if(ESP_OK != init_camera()) {
        return;
    }
#endif	

	// Flash Led Configuration:
	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

	//pin micro-ros task in APP_CPU  (Core 1)
	xTaskCreatePinnedToCore(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 1);			
}