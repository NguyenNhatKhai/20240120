/*
 * arm.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Nhat Khai
 */

#include "arm.h"

arm arms[ARM_NUMBER];

void armInit(void) {
	for (uint8_t i = 0; i < ARM_NUMBER; i ++) {
		arms[i].gripperID = SERVO_UNDEFINED;
		arms[i].wristID = SERVO_UNDEFINED;
		arms[i].elbowID = SERVO_UNDEFINED;
		arms[i].distance = 0;
		arms[i].degree = 0;
		arms[i].location = 0;
	}
}

uint8_t armStart(uint8_t gripperID, uint8_t wristID, uint8_t elbowID) {
	for (uint8_t i = 0; i < ARM_NUMBER; i ++) {
		if (arms[i].gripperID != SERVO_UNDEFINED || arms[i].wristID != SERVO_UNDEFINED || arms[i].elbowID != SERVO_UNDEFINED) continue;
		arms[i].gripperID = gripperID;
		arms[i].wristID = wristID;
		arms[i].elbowID = elbowID;
		arms[i].distance = ARM_RELAX;
		arms[i].degree = ARM_NORTHWARD;
		arms[i].location = ARM_BACKWARD;
		return i;
	}
	return ARM_UNDEFINED;
}

void armRun(void *) {
	for (uint8_t i = 0; i < ARM_NUMBER; i ++) {
		if (arms[i].gripperID != SERVO_UNDEFINED) {
			armGrip(&arms[i]);
		}
		if (arms[i].wristID != SERVO_UNDEFINED) {
			armRotate(&arms[i]);
		}
		if (arms[i].elbowID != SERVO_UNDEFINED) {
			armMove(&arms[i]);
		}
	}
}

void armGrip(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	if (newArmPointer->gripperID == SERVO_UNDEFINED) return;
	float newDistance = newArmPointer->distance / 2.0 + GRIPPER_THICK;
	if (newDistance < GRIPPER_MINIMUM) newDistance = GRIPPER_MINIMUM;
	else if (newDistance > GRIPPER_MAXIMUM) newDistance = GRIPPER_MAXIMUM;
	float newTarget = 90 - acosf((powf(GRIPPER_CRANK, 2) + powf(newDistance, 2) - powf(GRIPPER_ROD, 2)) / (2 * GRIPPER_CRANK * newDistance)) / M_PI * 180.0;
	servos[newArmPointer->gripperID].target = newTarget;
	servoRotate(&servos[newArmPointer->gripperID]);
}

void armRotate(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	if (newArmPointer->wristID == SERVO_UNDEFINED) return;
	float newDegree = newArmPointer->degree;
	if (newDegree < WRIST_MINIMUM) newDegree = WRIST_MINIMUM;
	else if (newDegree > WRIST_MAXIMUM) newDegree = WRIST_MAXIMUM;
	float newTarget = newDegree;
	servos[newArmPointer->wristID].target = newTarget;
	servoRotate(&servos[newArmPointer->wristID]);
}

void armMove(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	if (newArmPointer->elbowID == SERVO_UNDEFINED) return;
	float newLocation = newArmPointer->location;
	if (newLocation < ELBOW_MINIMUM) newLocation = ELBOW_MINIMUM;
	else if (newLocation > ELBOW_MAXIMUM) newLocation = ELBOW_MAXIMUM;
	float tempValue = (powf(ELBOW_CRANK, 2) - powf(ELBOW_ROD, 2) + powf(ELBOW_OFFSET, 2) + powf(newLocation, 2)) / (2 * ELBOW_CRANK);
	float newTarget = 180 - (2 * atanf((ELBOW_OFFSET + sqrtf(powf(ELBOW_OFFSET, 2) + powf(newLocation, 2) - pow(tempValue, 2))) / (newLocation + tempValue)) / M_PI * 180.0);
	servos[newArmPointer->elbowID].target = newTarget;
	servoRotate(&servos[newArmPointer->elbowID]);
}

void armHold(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->distance = ARM_HOLD;
	armGrip(newArmPointer);
}

void armRelease(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->distance = ARM_RELEASE;
	armGrip(newArmPointer);
}

void armRelax(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->distance = ARM_RELAX;
	armGrip(newArmPointer);
}

void armWestward(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->degree = ARM_WESTWARD;
	armRotate(newArmPointer);
}

void armNorthwest(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->degree = ARM_NORTHWEST;
	armRotate(newArmPointer);
}

void armNorthward(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->degree = ARM_NORTHWARD;
	armRotate(newArmPointer);
}

void armNortheast(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->degree = ARM_NORTHEAST;
	armRotate(newArmPointer);
}

void armEastward(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->degree = ARM_EASTWARD;
	armRotate(newArmPointer);
}

void armForward(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->location = ARM_FORWARD;
	armMove(newArmPointer);
}

void armBackward(void *armPointer) {
	arm *newArmPointer = (arm *)armPointer;
	newArmPointer->location = ARM_BACKWARD;
	armMove(newArmPointer);
}
