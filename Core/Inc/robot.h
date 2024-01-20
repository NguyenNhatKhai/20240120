/*
 * robot.h
 *
 *  Created on: Jan 17, 2024
 *      Author: Nhat Khai
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "main.h"
#include "scheduler.h"
#include "arm.h"
#include "servo.h"

typedef enum {
	INIT,
	FREE,
	BUSY
} robotState;

typedef struct {
	uint8_t armID;
} robotMovingArm;

typedef struct {
	uint8_t firstArmID;
	uint8_t secondArmID;
} robotFlippingArm;

void robotInit(void);

void robotBoot(void *);

void robotMoveLeftNormal(void *);
void robotMoveLeftInvert(void *);
void robotMoveLeftDouble(void *);
void robotMoveFrontNormal(void *);
void robotMoveFrontInvert(void *);
void robotMoveFrontDouble(void *);
void robotMoveRightNormal(void *);
void robotMoveRightInvert(void *);
void robotMoveRightDouble(void *);
void robotMoveBackNormal(void *);
void robotMoveBackInvert(void *);
void robotMoveBackDouble(void *);

void robotFlipXNormal(void *);
void robotFlipXInvert(void *);
void robotFlipXDouble(void *);
void robotFlipZNormal(void *);
void robotFlipZInvert(void *);
void robotFlipZDouble(void *);

void robotTest(void *);

#endif /* INC_ROBOT_H_ */