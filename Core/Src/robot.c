/*
 * robot.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Nhat Khai
 */

#include "robot.h"

static uint8_t leftArm = ARM_UNDEFINED;
static uint8_t frontArm = ARM_UNDEFINED;
static uint8_t rightArm = ARM_UNDEFINED;
static uint8_t backArm = ARM_UNDEFINED;

static robotState state;
static robotTurningArm turningArm;
static robotFlippingArm flippingArm;

void robotFree(void *) {
	state = FREE;
	turningArm.armID = ARM_UNDEFINED;
	flippingArm.firstArmID = ARM_UNDEFINED;
	flippingArm.secondArmID = ARM_UNDEFINED;
	flippingArm.thirdArmID = ARM_UNDEFINED;
	flippingArm.fourthArmID = ARM_UNDEFINED;
}

void robotInit(void) {
	state = INIT;
	turningArm.armID = ARM_UNDEFINED;
	flippingArm.firstArmID = ARM_UNDEFINED;
	flippingArm.secondArmID = ARM_UNDEFINED;
	flippingArm.thirdArmID = ARM_UNDEFINED;
	flippingArm.fourthArmID = ARM_UNDEFINED;

	servoInit();
	armInit();

	uint8_t leftGripper = servoStart(&htim2, TIM_CHANNEL_1, 0);
	uint8_t frontGripper = servoStart(&htim2, TIM_CHANNEL_2, 0);
	uint8_t rightGripper = servoStart(&htim2, TIM_CHANNEL_3, -5);
	uint8_t backGripper = servoStart(&htim2, TIM_CHANNEL_4, 0);

	uint8_t leftWrist = servoStart(&htim3, TIM_CHANNEL_1, -5);
	uint8_t frontWrist = servoStart(&htim3, TIM_CHANNEL_2, 0);
	uint8_t rightWrist = servoStart(&htim3, TIM_CHANNEL_3, -4);
	uint8_t backWrist = servoStart(&htim3, TIM_CHANNEL_4, 2);

	uint8_t leftElbow = servoStart(&htim4, TIM_CHANNEL_1, 0);
	uint8_t frontElbow = servoStart(&htim4, TIM_CHANNEL_2, 0);
	uint8_t rightElbow = servoStart(&htim4, TIM_CHANNEL_3, 0);
	uint8_t backElbow = servoStart(&htim4, TIM_CHANNEL_4, 10);

	leftArm = armStart(leftGripper, leftWrist, leftElbow);
	frontArm = armStart(frontGripper, frontWrist, frontElbow);
	rightArm = armStart(rightGripper, rightWrist, rightElbow);
	backArm = armStart(backGripper, backWrist, backElbow);
}

void robotBoot(void *) {
	schedulerAddTask(armRun, NULL, 1 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armRelease, &arms[rightArm], 2 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armHold, &arms[rightArm], 3 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armForward, &arms[rightArm], 4 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armForward, &arms[leftArm], 5 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armForward, &arms[frontArm], 6 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armForward, &arms[backArm], 6 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armHold, &arms[leftArm], 7 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armHold, &arms[frontArm], 7 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(armHold, &arms[backArm], 7 * ROBOT_MANUALDURATION, 0);
	schedulerAddTask(robotFree, NULL, 8 * ROBOT_MANUALDURATION, 0);
}

void robotShutDown(void *) {
	schedulerAddTask(armRelease, &arms[leftArm], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[frontArm], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[rightArm], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[backArm], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[leftArm], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[frontArm], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[rightArm], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[backArm], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armNorthward, &arms[leftArm], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armNorthward, &arms[frontArm], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armNorthward, &arms[rightArm], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armNorthward, &arms[backArm], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelax, &arms[leftArm], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelax, &arms[frontArm], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelax, &arms[rightArm], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelax, &arms[backArm], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotFree, NULL, 4 * ROBOT_AUTODURATION, 0);
}

void robotTurnReturn(void *) {
	schedulerAddTask(armRelease, &arms[turningArm.armID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[turningArm.armID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armNorthward, &arms[turningArm.armID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[turningArm.armID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[turningArm.armID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotFree, NULL, 5 * ROBOT_AUTODURATION, 0);
}

void robotTurnNormal(void *) {
	schedulerAddTask(armRelease, &arms[turningArm.armID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armFirstLayer, &arms[turningArm.armID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[turningArm.armID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armWestward, &arms[turningArm.armID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotTurnReturn, NULL, 4 * ROBOT_AUTODURATION, 0);
}

void robotTurnInvert(void *) {
	schedulerAddTask(armRelease, &arms[turningArm.armID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armFirstLayer, &arms[turningArm.armID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[turningArm.armID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armEastward, &arms[turningArm.armID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotTurnReturn, NULL, 4 * ROBOT_AUTODURATION, 0);
}

void robotTurnDouble(void *) {
	schedulerAddTask(armRelease, &arms[turningArm.armID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[turningArm.armID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armWestward, &arms[turningArm.armID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armFirstLayer, &arms[turningArm.armID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[turningArm.armID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armEastward, &arms[turningArm.armID], 5 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotTurnReturn, NULL, 6 * ROBOT_AUTODURATION, 0);
}

void robotTurnLeftNormal(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = leftArm;
	robotTurnNormal(NULL);
}

void robotTurnLeftInvert(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = leftArm;
	robotTurnInvert(NULL);
}

void robotTurnLeftDouble(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = leftArm;
	robotTurnDouble(NULL);
}

void robotTurnFrontNormal(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = frontArm;
	robotTurnNormal(NULL);
}

void robotTurnFrontInvert(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = frontArm;
	robotTurnInvert(NULL);
}

void robotTurnFrontDouble(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = frontArm;
	robotTurnDouble(NULL);
}

void robotTurnRightNormal(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = rightArm;
	robotTurnNormal(NULL);
}

void robotTurnRightInvert(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = rightArm;
	robotTurnInvert(NULL);
}

void robotTurnRightDouble(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = rightArm;
	robotTurnDouble(NULL);
}

void robotTurnBackNormal(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = backArm;
	robotTurnNormal(NULL);
}

void robotTurnBackInvert(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = backArm;
	robotTurnInvert(NULL);
}

void robotTurnBackDouble(void *) {
	if (state != FREE) return;
	state = BUSY;
	turningArm.armID = backArm;
	robotTurnDouble(NULL);
}

void robotFlipReturn(void *) {
	schedulerAddTask(armRelease, &arms[flippingArm.firstArmID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[flippingArm.secondArmID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.firstArmID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.secondArmID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armNorthward, &arms[flippingArm.firstArmID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armNorthward, &arms[flippingArm.secondArmID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.firstArmID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.secondArmID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.firstArmID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.secondArmID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotFree, NULL, 5 * ROBOT_AUTODURATION, 0);
}

void robotFlipSingle(void *) {
	schedulerAddTask(armRelease, &arms[flippingArm.thirdArmID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[flippingArm.fourthArmID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.thirdArmID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.fourthArmID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armEastward, &arms[flippingArm.firstArmID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armWestward, &arms[flippingArm.secondArmID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.thirdArmID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.fourthArmID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.thirdArmID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.fourthArmID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotFlipReturn, NULL, 5 * ROBOT_AUTODURATION, 0);
}

void robotFlipDouble(void *) {
	schedulerAddTask(armRelease, &arms[flippingArm.firstArmID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[flippingArm.secondArmID], 0 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.firstArmID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.secondArmID], 1 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armWestward, &arms[flippingArm.firstArmID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armEastward, &arms[flippingArm.secondArmID], 2 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.firstArmID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.secondArmID], 3 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.firstArmID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.secondArmID], 4 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[flippingArm.thirdArmID], 5 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armRelease, &arms[flippingArm.fourthArmID], 5 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.thirdArmID], 6 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armBackward, &arms[flippingArm.fourthArmID], 6 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armEastward, &arms[flippingArm.firstArmID], 7 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armWestward, &arms[flippingArm.secondArmID], 7 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.thirdArmID], 8 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armForward, &arms[flippingArm.fourthArmID], 8 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.thirdArmID], 9 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(armHold, &arms[flippingArm.fourthArmID], 9 * ROBOT_AUTODURATION, 0);
	schedulerAddTask(robotFlipReturn, NULL, 10 * ROBOT_AUTODURATION, 0);
}

void robotFlipXNormal(void *) {
	if (state != FREE) return;
	state = BUSY;
	flippingArm.firstArmID = rightArm;
	flippingArm.secondArmID = leftArm;
	flippingArm.thirdArmID = frontArm;
	flippingArm.fourthArmID = backArm;
	robotFlipSingle(NULL);
}

void robotFlipXInvert(void *) {
	if (state != FREE) return;
	state = BUSY;
	flippingArm.firstArmID = leftArm;
	flippingArm.secondArmID = rightArm;
	flippingArm.thirdArmID = frontArm;
	flippingArm.fourthArmID = backArm;
	robotFlipSingle(NULL);
}

void robotFlipXDouble(void *) {
	if (state != FREE) return;
	state = BUSY;
	flippingArm.firstArmID = rightArm;
	flippingArm.secondArmID = leftArm;
	flippingArm.thirdArmID = frontArm;
	flippingArm.fourthArmID = backArm;
	robotFlipDouble(NULL);
}

void robotFlipZNormal(void *) {
	if (state != FREE) return;
	state = BUSY;
	flippingArm.firstArmID = frontArm;
	flippingArm.secondArmID = backArm;
	flippingArm.thirdArmID = leftArm;
	flippingArm.fourthArmID = rightArm;
	robotFlipSingle(NULL);
}

void robotFlipZInvert(void *) {
	if (state != FREE) return;
	state = BUSY;
	flippingArm.firstArmID = backArm;
	flippingArm.secondArmID = frontArm;
	flippingArm.thirdArmID = leftArm;
	flippingArm.fourthArmID = rightArm;
	robotFlipSingle(NULL);
}

void robotFlipZDouble(void *) {
	if (state != FREE) return;
	state = BUSY;
	flippingArm.firstArmID = frontArm;
	flippingArm.secondArmID = backArm;
	flippingArm.thirdArmID = leftArm;
	flippingArm.fourthArmID = rightArm;
	robotFlipDouble(NULL);
}

void robotTest(void *) {
	schedulerAddTask(armEastward, &arms[rightArm], 2000, 0);
	schedulerAddTask(armNorthward, &arms[rightArm], 4000, 0);
	schedulerAddTask(armWestward, &arms[rightArm], 6000, 0);
}
