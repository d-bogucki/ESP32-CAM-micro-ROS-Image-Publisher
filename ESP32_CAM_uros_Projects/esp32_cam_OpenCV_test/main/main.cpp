//===========================================================================================================================================================================
// This application does some benchmarking of pre-build OpenCV library functions inside a ESP32 MCU
// In particular the computational time of specific functions used inside the "detectMarkers()" of the
// ArUco module are measured.
// Additionally for a visual check, the transformed and compressed images are then published on a ROS2 topic.
//===========================================================================================================================================================================

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// OpenCV Modules Headers:
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#undef EPS
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Standard Library Headers
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <string.h>
#include <stdio.h>
#include <unistd.h>
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

extern "C" {
void app_main(void);
}

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
static const char *TAG = "ESP32_OpenCV_Test_Publisher";

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
	.frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA Do not use sizes above QVGA when not JPEG
	// Other frame sizes:
	// FRAMESIZE_QVGA --> 320x240
	// FRAMESIZE_CIF  --> 400x296
	// FRAMESIZE_VGA  --> 640x480
	// FRAMESIZE_SVGA --> 800x600
	// FRAMESIZE_HD   --> 1280x720
	// FRAMESIZE UXGA --> 1600x1200 (2 MP maximum resolution)
	.jpeg_quality = 15, 	                // 0-63 lower number means higher quality
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

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Timer Callback function Definition
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{	

// Time variables for logs in debug mode:	
int64_t time_0 = esp_timer_get_time();	// [us]
int64_t elapsed_time;

	
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {

		// Time Log:
		elapsed_time = (time_0 - last_callback)/1000; 	// [ms]
		ESP_LOGI(TAG, "Callback Init! Time elapsed between the last callback start : %lld [ms]\n", elapsed_time);
		last_callback = time_0;		
		
		// Getting the pointer to a frame buffer:
		camera_fb_t *pic = esp_camera_fb_get();			// use pic->buf to access the image	

		if (pic != NULL) {

            // Logging Image size and location:
            ESP_LOGI(TAG, "Image captured! Size: %zu [bytes], Resolution: (%d x %d)\n",  
                pic->len, pic->width, pic->height);   

			// Logging the msg_static capacity:
			ESP_LOGI(TAG, "the msg_static capacity is : %d bytes \n", msg_static.data.capacity);  	

			if (pic->len <= msg_static.data.capacity) { 
				
                // Dynamic memory allocation for BGR565 Image buffer:
                size_t rgb565_img_size = (pic->height)*(pic->width)*2;          // 2 bytes every pixel
                uint8_t *rgb565_img_ptr = (uint8_t *)malloc(rgb565_img_size);   // image pointer
                jpg_scale_t scale = JPG_SCALE_NONE;

                // Converting the JPEG frame buffer into bgr565 image through camera drivers conversion fucntions:
                int64_t time_1 = esp_timer_get_time();
                jpg2rgb565(pic->buf, pic->len, rgb565_img_ptr, scale);
                int64_t time_2 = esp_timer_get_time();

                ESP_LOGI(TAG, "the jpg2rgb565() function took %lld [ms]\n", (time_2 - time_1)/1000);

                // Creating a "cv::Mat" for the rgb565 image:
                cv::Mat cv_img(pic->height, pic->width, CV_8UC2, rgb565_img_ptr);

                // Dynamic memory allocation for grayscale image buffer:
                size_t gray_img_size = (pic->height)*(pic->width);              // 1 byte per pixel
                uint8_t *gray_img_ptr = (uint8_t *)malloc(gray_img_size);

                // Creating a "cv::Mat" for the gray image:
                cv::Mat cv_gray_img(pic->height, pic->width, CV_8UC1, gray_img_ptr);

                //---------------------------------------------------------------------------------------------------------------
                // cv::cvtColor() Benchmarking:
                //---------------------------------------------------------------------------------------------------------------

                int64_t time_3 = esp_timer_get_time();
                cv::cvtColor(cv_img, cv_gray_img, cv::COLOR_BGR5652GRAY);
                int64_t time_4 = esp_timer_get_time();

                ESP_LOGI(TAG, "cvtColor() function took %lld [ms]\n", (time_4 - time_3)/1000);

                //---------------------------------------------------------------------------------------------------------------
                // cv::threshold() Benchmarking:
                //---------------------------------------------------------------------------------------------------------------

                int64_t time_5 = esp_timer_get_time();
                cv::threshold(cv_gray_img, cv_gray_img, 128, 255, cv::THRESH_OTSU);
                int64_t time_6 = esp_timer_get_time();

                ESP_LOGI(TAG, "threshold() function with OTSU algorithm took %lld [ms]\n", (time_6 - time_5)/1000);

                //---------------------------------------------------------------------------------------------------------------
                // cv::findcontours() Benchmarking:
                //---------------------------------------------------------------------------------------------------------------
                
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;

                int64_t time_7 = esp_timer_get_time();
                cv::findContours(cv_gray_img, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
                int64_t time_8 = esp_timer_get_time();

                ESP_LOGI(TAG, "findcontours() function took %lld [ms]\n", (time_8 - time_7)/1000);

                //---------------------------------------------------------------------------------------------------------------
                // cv::approxPolyDP() Benchmarking:
                //---------------------------------------------------------------------------------------------------------------
                
                uint16_t min_Perimeter_Pixel = 100; 
                std::vector<cv::Point> approximated_curve;   
                double accuracy_rate = 0.05;                // accuracy rate for the approximated polygon 

                int64_t time_9 = esp_timer_get_time();
                for (int i = 0; i < contours.size(); i++) {
                    
                    // Identified Countours with less than 100 pixel of perimeters won't be processed:
                    if (contours[i].size() < min_Perimeter_Pixel) continue;
                                    
                    // Approximate the curve with a polygon:
                    cv::approxPolyDP(contours[i], approximated_curve, double(contours[i].size() * accuracy_rate), true);

                    // Exit from the loop at the first approximated polygon with 4 vertices and convex shape:
                    if (approximated_curve.size() == 4 && cv::isContourConvex(approximated_curve)) break;
                }
                int64_t time_10 = esp_timer_get_time();

                ESP_LOGI(TAG, "approxPolyDP() function took %lld [ms]\n", (time_10 - time_9)/1000);
                
                //---------------------------------------------------------------------------------------------------------------
                // cv::warpPerspective() Benchmarking:
                //---------------------------------------------------------------------------------------------------------------

                // OSS: I wanted to test also the perspective distortion removing of the marker's identified contours, however there
                // seems to be an issue with the opencv's "imgwarp.cpp" file. In fact, inside it, some tables like NNDeltaTab, BicubicTab,
                // and Lanczos4Tab are statically allocated and causing a .bss segment of DRAM overflow of around 400 kb.
                // For solving the issue you should try to allocate them in external RAM using the EXTR_RAM_ATTR together with
                // enable the .bss in external RAM region using the "menuconfig", then re-build all the OpenCV library for the application.

                /*
                int marker_size_with_borders = 4 + 2*1;     // Considering a 4x4 ArUco dictionary with 1 bit for borders
                int cell_size = 40;                         // Hence a (240x240) output binary image
                int marker_img_size = marker_size_with_borders * cell_size;

                uint8_t *marker_img_ptr = (uint8_t *)malloc(marker_img_size*marker_img_size);       // marker image pointer
                cv::Mat marker_img(marker_img_size, marker_img_size, CV_8UC1, marker_img_ptr);
                
                // If at least one rectangular shape polygon was found, then we apply the perspective transform:
                if (!approximated_curve.empty()) {
                    
                    cv::Mat marker_img_corners(4, 1, CV_32FC2);
                    marker_img_corners.ptr<cv::Point2f>(0)[0] = cv::Point2f(0, 0);
                    marker_img_corners.ptr<cv::Point2f>(0)[1] = cv::Point2f((float)marker_img_size - 1, 0);
                    marker_img_corners.ptr<cv::Point2f>(0)[2] = cv::Point2f((float)marker_img_size - 1, (float)marker_img_size - 1);
                    marker_img_corners.ptr<cv::Point2f>(0)[3] = cv::Point2f(0, (float)marker_img_size - 1);

                    // You Need to verify that the corners are in a consistent order! (see "_reorderCandidatesCorners()" function)
                    // Check not present at the moment

                    
                    // Remove perspective:
                    //int64_t time_11 = esp_timer_get_time();
                    cv::Mat transformation(3, 3, CV_32F);
                    //transformation = cv::getPerspectiveTransform(approximated_curve, marker_img_corners);
                    //cv::warpPerspective(cv_img, marker_img, transformation, cv::Size(marker_img_size, marker_img_size), cv::INTER_NEAREST);
                    
                    
                    //int64_t time_12 = esp_timer_get_time();

                    //ESP_LOGI(TAG, "warpPerspective() function took %lld [ms]\n", (time_12 - time_11)/1000);
                    
                }
                */                
                
                //--------------------------------------------------------------------------------------------------------------
                // Converting the processed image (after the adaptive thresholding) back into jpeg format:

                size_t gray_img_len = (pic->width) * (pic->height);
                uint8_t *jpeg_out_buf = NULL;
                size_t _jpg_buf_len;

                fmt2jpg(gray_img_ptr, gray_img_len, pic->width, pic->height, PIXFORMAT_GRAYSCALE, 15, &jpeg_out_buf, &_jpg_buf_len);
                ESP_LOGI(TAG,"The size of the output jpeg image is: %zu\n", _jpg_buf_len);

                // Setting the valid number of elements of the image buffer:
                msg_static.data.size = _jpg_buf_len;

                // Copying the image buffer of the camera driver into the data field of the micro-ROS message:
                memcpy(msg_static.data.data, jpeg_out_buf, _jpg_buf_len); 

                // Setting other msg_static fields, header and format:
                msg_static.header.frame_id = micro_ros_string_utilities_set(msg_static.header.frame_id, "myframe");
                msg_static.format = micro_ros_string_utilities_set(msg_static.format, "jpeg");
                    
                // Message Publishing:	    
                RCSOFTCHECK(rcl_publish(&publisher, &msg_static, NULL));

                // Freeing the allocated image memory buffers:
                free(rgb565_img_ptr);
                free(jpeg_out_buf); 
                free(gray_img_ptr);
                //free(marker_img_ptr);
			}
			else
			{
				ESP_LOGE(TAG, "Image size exceeeds micro-ROS msg capacity, image size was: %zu [bytes], capacity is: %zu [bytes]\n", 
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

    // Get the amount of stack (in Bytes) that remained unused when the task stack was at its greatest value
	ESP_LOGI(TAG, "task stack watermark: %d Bytes\n", uxTaskGetStackHighWaterMark(NULL));
	// Get the free heap in Bytes (may not be contiguous)
	ESP_LOGI(TAG, "heap left: %lu Bytes\n\n", esp_get_free_heap_size());
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
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	// During the initialization the rclc executor dynamically allocates memory for the total number of "handles",
	// hence the number of subscription, timers, clients, services and so on.

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
		{"format", 4},	// since "jpeg" is the longest acceptable format
		{"data", 10000} // Allocates up to 10000 bytes for the data field (compressed image data) (equal to the MTU size) 
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

    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// SPIN LOOP:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	}

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// CLEAN UP:
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
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

	//pin micro-ros task in APP_CPU  (Core 1)
	xTaskCreatePinnedToCore(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 1);			
}

