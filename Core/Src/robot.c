/*
 * robot.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Nhat Khai
 */

#include "robot.h"

static uint8_t leftGripper = SERVO_UNDEFINED;
static uint8_t frontGripper = SERVO_UNDEFINED;
static uint8_t rightGripper = SERVO_UNDEFINED;
static uint8_t backGripper = SERVO_UNDEFINED;

static uint8_t leftWrist = SERVO_UNDEFINED;
static uint8_t frontWrist = SERVO_UNDEFINED;
static uint8_t rightWrist = SERVO_UNDEFINED;
static uint8_t backWrist = SERVO_UNDEFINED;

static uint8_t leftElbow = SERVO_UNDEFINED;
static uint8_t frontElbow = SERVO_UNDEFINED;
static uint8_t rightElbow = SERVO_UNDEFINED;
static uint8_t backElbow = SERVO_UNDEFINED;

static uint8_t leftArm = ARM_UNDEFINED;
static uint8_t frontArm = ARM_UNDEFINED;
static uint8_t rightArm = ARM_UNDEFINED;
static uint8_t backArm = ARM_UNDEFINED;

static robotState state;
static robotMovingArm movingArm;
static robotFlippingArm flippingArm;

void robotFree(void *) {
	state = FREE;
	movingArm.armID = ARM_UNDEFINED;
	flippingArm.firstArmID = ARM_UNDEFINED;
	flippingArm.secondArmID = ARM_UNDEFINED;
}

void robotInit(void) {
	state = INIT;
	movingArm.armID = ARM_UNDEFINED;
	flippingArm.firstArmID = ARM_UNDEFINED;
	flippingArm.secondArmID = ARM_UNDEFINED;

	servoInit();
	armInit();

	leftGripper = servoStart(&htim2, TIM_CHANNEL_1, 0);
	frontGripper = servoStart(&htim2, TIM_CHANNEL_2, 0);
	rightGripper = servoStart(&htim2, TIM_CHANNEL_3, 0);
	backGripper = servoStart(&htim2, TIM_CHANNEL_4, 0);

	leftWrist = servoStart(&htim3, TIM_CHANNEL_1, 0);
	frontWrist = servoStart(&htim3, TIM_CHANNEL_2, 0);
	rightWrist = servoStart(&htim3, TIM_CHANNEL_3, 0);
	backWrist = servoStart(&htim3, TIM_CHANNEL_4, 0);

	leftElbow = servoStart(&htim4, TIM_CHANNEL_1, 0);
	frontElbow = servoStart(&htim4, TIM_CHANNEL_2, 0);
	rightElbow = servoStart(&htim4, TIM_CHANNEL_3, 0);
	backElbow = servoStart(&htim4, TIM_CHANNEL_4, 0);

	leftArm = armStart(leftGripper, leftWrist, leftElbow);
	frontArm = armStart(frontGripper, frontWrist, frontElbow);
	rightArm = armStart(rightGripper, rightWrist, rightElbow);
	backArm = armStart(backGripper, backWrist, backElbow);
}

void robotBoot(void *) {
	schedulerAddTask(servoRun, NULL, 1000, 0);
	schedulerAddTask(armRun, NULL, 2000, 0);
	schedulerAddTask(armHold, &arms[rightArm], 3000, 0);
	schedulerAddTask(armForward, &arms[leftArm], 4000, 0);
	schedulerAddTask(armForward, &arms[frontArm], 4000, 0);
	schedulerAddTask(armForward, &arms[rightArm], 4000, 0);
	schedulerAddTask(armForward, &arms[backArm], 4000, 0);
	schedulerAddTask(armHold, &arms[leftArm], 5000, 0);
	schedulerAddTask(armHold, &arms[frontArm], 5000, 0);
	schedulerAddTask(armHold, &arms[backArm], 5000, 0);
	schedulerAddTask(robotFree, NULL, 6000, 0);
}

void robotMoveReturn(void *) {
	schedulerAddTask(armRelease, &arms[movingArm.armID], 1000, 0);
	schedulerAddTask(armBackward, &arms[movingArm.armID], 2000, 0);
	schedulerAddTask(armNorthward, &arms[movingArm.armID], 3000, 0);
	schedulerAddTask(armForward, &arms[movingArm.armID], 4000, 0);
	schedulerAddTask(armHold, &arms[movingArm.armID], 5000, 0);
	schedulerAddTask(robotFree, NULL, 6000, 0);
}

void robotMoveNormal(void *) {
	if (state != FREE) return;
	state = BUSY;
	schedulerAddTask(armEastward, &arms[movingArm.armID], 1000, 0);
	schedulerAddTask(robotMoveReturn, NULL, 2000, 0);
}

void robotMoveInvert(void *) {
	if (state != FREE) return;
	state = BUSY;
	schedulerAddTask(armWestward, &arms[movingArm.armID], 1000, 0);
	schedulerAddTask(robotMoveReturn, NULL, 2000, 0);
}

void robotMoveDouble(void *) {
	if (state != FREE) return;
	state = BUSY;
	schedulerAddTask(armRelease, &arms[movingArm.armID], 1000, 0);
	schedulerAddTask(armBackward, &arms[movingArm.armID], 2000, 0);
	schedulerAddTask(armEastward, &arms[movingArm.armID], 3000, 0);
	schedulerAddTask(armForward, &arms[movingArm.armID], 4000, 0);
	schedulerAddTask(armHold, &arms[movingArm.armID], 5000, 0);
	schedulerAddTask(armWestward, &arms[movingArm.armID], 6000, 0);
	schedulerAddTask(robotMoveReturn, NULL, 7000, 0);
}

void robotMoveLeftNormal(void *) {
	movingArm.armID = leftArm;
	schedulerAddTask(robotMoveNormal, NULL, 1000, 0);
}

void robotMoveLeftInvert(void *) {
	movingArm.armID = leftArm;
	schedulerAddTask(robotMoveInvert, NULL, 1000, 0);
}

void robotMoveLeftDouble(void *) {
	movingArm.armID = leftArm;
	schedulerAddTask(robotMoveDouble, NULL, 1000, 0);
}

void robotMoveFrontNormal(void *) {
	movingArm.armID = frontArm;
	schedulerAddTask(robotMoveNormal, NULL, 1000, 0);
}

void robotMoveFrontInvert(void *) {
	movingArm.armID = frontArm;
	schedulerAddTask(robotMoveInvert, NULL, 1000, 0);
}

void robotMoveFrontDouble(void *) {
	movingArm.armID = frontArm;
	schedulerAddTask(robotMoveDouble, NULL, 1000, 0);
}

void robotMoveRightNormal(void *) {
	movingArm.armID = rightArm;
	schedulerAddTask(robotMoveNormal, NULL, 1000, 0);
}

void robotMoveRightInvert(void *) {
	movingArm.armID = rightArm;
	schedulerAddTask(robotMoveInvert, NULL, 1000, 0);
}

void robotMoveRightDouble(void *) {
	movingArm.armID = rightArm;
	schedulerAddTask(robotMoveDouble, NULL, 1000, 0);
}

void robotMoveBackNormal(void *) {
	movingArm.armID = backArm;
	schedulerAddTask(robotMoveNormal, NULL, 1000, 0);
}

void robotMoveBackInvert(void *) {
	movingArm.armID = backArm;
	schedulerAddTask(robotMoveInvert, NULL, 1000, 0);
}

void robotMoveBackDouble(void *) {
	movingArm.armID = backArm;
	schedulerAddTask(robotMoveDouble, NULL, 1000, 0);
}

uint8_t i = 0;
void robotTest(void *) {
	uint8_t j = leftGripper;
	if (i == 0) {
		servos[j].target = 0;
		i = 1;
	}
	else if (i == 1) {
		servos[j].target = 90;
		i = 2;
	}
	else if (i == 2) {
		servos[j].target = 0;
		i = 0;
	}
	else {
		i = 0;
	}
	schedulerAddTask(servoRotate, &servos[j], 0, 0);
}