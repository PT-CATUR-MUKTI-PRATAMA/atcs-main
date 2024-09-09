/*
 * RS485.h
 *
 *  Created on: Apr 30, 2024
 *      Author: mohhi
 */

#ifndef INC_RS485_DRIVER_H_
#define INC_RS485_DRIVER_H_

void RequestData_Driver(UART_HandleTypeDef *huart, uint8_t driver_id);
void Receive_485(UART_HandleTypeDef *huart);

extern char rxData;
extern uint16_t I_LAMP[36];

#endif /* INC_RS485_DRIVER_H_ */
