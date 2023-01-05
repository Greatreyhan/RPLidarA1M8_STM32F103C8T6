/*
 * RPLidarA1.h
 *
 *  Created on: Des 25, 2022
 *      Author: Maulana Reyhan Savero
 */
 #include "main.h"
 
 #ifndef RPLIDARA1_H_
 #define RPLIDARA1_H_
 
 #define LIDAR_START_FLAG 0xa5
#define LIDAR_NEXT_FLAG 0x5a

// Send Mode
#define LIDAR_SEND_SINGLE_RESPONSE 0x0
#define LIDAR_SEND_MULTIPLE_RESPONSE 0x1

// Command
#define LIDAR_STOP 0x25
#define LIDAR_RESET 0x40
#define LIDAR_SCAN 0x20
#define LIDAR_EXPRESS_SCAN 0x82
#define LIDAR_FORCE_SCAN 0x21
#define LIDAR_GET_INFO 0x50
#define LIDAR_GET_HEALTH 0x52
#define LIDAR_GET_SAMPLERATE 0x59
#define LIDAR_GET_CONF 0x84

// Size of Data Received
#define LIDAR_SIZE_GET_SCAN 5 // bytes
#define LIDAR_SIZE_GET_FORCE_SCAN 5
#define LIDAR_SIZE_GET_INFO 20
#define LIDAR_SIZE_GET_HEALTH 3
#define LIDAR_SIZE_GET_SAMPLERATE 4

// Lidar Conf Command
#define LIDAR_CONF_SCAN_MODE_COUNT 0x70
#define LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE 0x71
#define LIDAR_CONF_SCAN_MODE_MAX_DISTANCE 0x74
#define LIDAR_CONF_SCAN_MODE_ANS_TYPE 0x75
#define LIDAR_CONF_SCAN_MODE_TYPICAL 0x7C
#define LIDAR_CONF_SCAN_MODE_NAME 0x7F

// Nodes Reading
#define LIDAR_MAX_WRONG_NODES 20

typedef enum
{
  LIDAR_OK       = 0x00U,
  LIDAR_TIMEOUT    = 0x01U,
  LIDAR_NO_GOOD_ANS     = 0x02U,
  LIDAR_NO_GOOD_NODE  = 0x03U
} lidar_StatusTypeDef;

typedef struct __lidar_rec_Handle
{	/*utilities */
	UART_HandleTypeDef* huart;
	uint8_t buff[30];
	/* data  */
	uint8_t descriptor[7];
	uint8_t start_scan_flag;
	uint8_t quality;
	float angle;
	float distance;
} lidar_HandleTypeDef;

typedef struct{
	uint8_t descriptor[7];
	uint8_t Model;
	uint8_t Firmware_Minor;
	uint8_t Firmware_Major;
	uint8_t Hardware;
	uint8_t Serial_Number[16];
} lidar_info_response_t;

typedef struct{
	uint8_t descriptor[7];
	uint8_t Status;
	uint16_t Error_Code;
} lidar_health_response_t;


typedef struct{
	uint8_t descriptor[7];
	uint16_t Time_Standart;
	uint16_t Time_Express;
} lidar_samplerate_response_t;

// function
void lidar_setup(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_start_scan(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_read_scan(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_get_point(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_read_node(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_check_node(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_get_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health);
lidar_StatusTypeDef lidar_read_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health);
lidar_StatusTypeDef lidar_get_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info);
lidar_StatusTypeDef lidar_read_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info);
lidar_StatusTypeDef lidar_get_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate);
lidar_StatusTypeDef lidar_read_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate);

 #endif
