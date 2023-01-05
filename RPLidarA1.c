/*
 * RPLidarA1.c
 *
 *  Created on: Des 25, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "RPLidarA1.h"

void lidar_setup(lidar_HandleTypeDef* lidar){
	uint8_t rplidar_stop_msg[] = {LIDAR_START_FLAG,LIDAR_STOP};
	
	HAL_UART_Transmit(lidar->huart, rplidar_stop_msg, 2, 1000);
	HAL_Delay(20);
}

lidar_StatusTypeDef lidar_start_scan(lidar_HandleTypeDef* lidar){
	uint8_t rplidar_scan_msg[] = {0xA5, 0x20};
	
	HAL_UART_Transmit(lidar->huart, rplidar_scan_msg, 2, 1000);
	uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 7, 1000);
	if(resp == HAL_OK){
		return lidar_read_scan(lidar);
	}
	return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_scan(lidar_HandleTypeDef* lidar){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x05 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x40 && lidar->buff[6]==0x81){
		for(uint8_t i =0; i < 7; i++){
			lidar->descriptor[i] = lidar->buff[i];
		}
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

lidar_StatusTypeDef lidar_check_node(lidar_HandleTypeDef* lidar){
	if((lidar->buff[1]&0x01)&((lidar->buff[0]^(lidar->buff[0]>>1))&0x01)){
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_NODE;
}

lidar_StatusTypeDef lidar_read_node(lidar_HandleTypeDef* lidar){
	uint8_t status = HAL_UART_Receive(lidar->huart, lidar->buff, 5, 1000);
	if(status == HAL_OK){
		return lidar_check_node(lidar);
	}
	return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_get_point(lidar_HandleTypeDef* lidar){
	lidar_StatusTypeDef status;
	uint8_t wrong_nodes = 0;
	do{
		status = lidar_read_node(lidar);
		wrong_nodes++;
	}
	while((status==LIDAR_NO_GOOD_NODE)&&(wrong_nodes<LIDAR_MAX_WRONG_NODES));
	
	if(status == LIDAR_OK){
		lidar->angle = ((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0; //deg
		lidar->distance = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/1000; // meters
		lidar->start_scan_flag = lidar->buff[0] & 0x1;
		lidar->quality = lidar->buff[0]>>2;
	}
	
	return status;
}

lidar_StatusTypeDef lidar_get_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health){
		uint8_t rplidar_health_msg[] = {0xA5, 0x52};
		HAL_UART_Transmit(lidar->huart, rplidar_health_msg, 2, 1000);
		uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 10, 1000);
		if(resp == HAL_OK){
			return lidar_read_health(lidar, health);
		}
		return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x03 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x00 && lidar->buff[6]==0x06){
		for(uint8_t i =0; i < 7; i++){
			health->descriptor[i] = lidar->buff[i];
		}
		health->Status =  lidar->buff[7];
		health->Error_Code = lidar->buff[8] | (lidar->buff[9] << 8);
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

lidar_StatusTypeDef lidar_get_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info){
	uint8_t rplidar_info_msg[] = {0xA5, 0x50};
	HAL_UART_Transmit(lidar->huart, rplidar_info_msg, 2, 1000);
	uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 27, 1000);
	if(resp == HAL_OK){
		return lidar_read_info(lidar,info);
	}
	return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x14 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x00 && lidar->buff[6]==0x04){
		for(uint8_t i =0; i < 7; i++){
			info->descriptor[i] = lidar->buff[i];
		}
		info->Model = lidar->buff[7];
		info->Firmware_Minor = lidar->buff[8];
		info->Firmware_Major = lidar->buff[9];
		info->Hardware = lidar->buff[10];
		for(uint8_t i=0;i<16;i++){
			info->Serial_Number[i] = lidar->buff[11+i];
		}
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

lidar_StatusTypeDef lidar_get_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate){
		uint8_t rplidar_samplerate_msg[] = {0xA5, 0x59};
		HAL_UART_Transmit(lidar->huart, rplidar_samplerate_msg, 2, 1000);
		uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 11, 1000);
		if(resp == HAL_OK){
			return lidar_read_samplerate(lidar, samplerate);
		}
		return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x04 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x00 && lidar->buff[6]==0x15){
		for(uint8_t i =0; i < 7; i++){
			samplerate->descriptor[i] = lidar->buff[i];
		}
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

