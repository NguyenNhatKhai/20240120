/*
 * servo.h
 *
 *  Created on: Jan 16, 2024
 *      Author: Nhat Khai
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

#define SERVO_NUMBER 12
#define SERVO_UNDEFINED SERVO_NUMBER

#define SERVO_PERIOD 20000				// in microseconds
#define SERVO_MINIMUM 500				// in microseconds
#define SERVO_MAXIMUM 2500				// in microseconds
#define SERVO_OFFSET 13.5				// in degrees

typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	float offset;
	float target;
} servo;

extern servo servos[SERVO_NUMBER];

void servoInit(void);
uint8_t servoStart(TIM_HandleTypeDef *timer, uint32_t channel, float offset);

void servoRun(void *);
void servoRotate(void *servoPointer);

#endif /* INC_SERVO_H_ */
