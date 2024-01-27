/*
 * scorpions.c
 *
 *  Created on: Jan 23, 2024
 *      Author: Nhat Khai
 */

#include "scorpions.h"

static uint8_t scramble = 10;
//static scorpionsMove scrambles[2] = {LN, RN};
static scorpionsMove scrambles[10] = {L2, UN, F2, DN, B2, UN, B2, UN, F2, UI};
//static scorpionsMove scrambles[21] = {FN, BI, R2, LI, UN, BI, U2, FN, BN, D2, RI, U2, F2, D2, L2, F2, L2, DI, F2, R2, U2};

void scorpionsScramble(void *) {
	uint32_t newDelay = 1000;
	for (uint8_t i = 0; i < scramble; i ++) {
		switch (scrambles[i]) {
		case UN:
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotTurnFrontNormal, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case UI:
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotTurnFrontInvert, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case U2:
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotTurnFrontDouble, NULL, newDelay, 0);
			newDelay += ROBOT_TURNDOUBLE;
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case LN:
			schedulerAddTask(robotTurnLeftNormal, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case LI:
			schedulerAddTask(robotTurnLeftInvert, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case L2:
			schedulerAddTask(robotTurnLeftDouble, NULL, newDelay, 0);
			newDelay += ROBOT_TURNDOUBLE;
			break;
		case FN:
			schedulerAddTask(robotTurnFrontNormal, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case FI:
			schedulerAddTask(robotTurnFrontInvert, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case F2:
			schedulerAddTask(robotTurnFrontDouble, NULL, newDelay, 0);
			newDelay += ROBOT_TURNDOUBLE;
			break;
		case RN:
			schedulerAddTask(robotTurnRightNormal, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case RI:
			schedulerAddTask(robotTurnRightInvert, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case R2:
			schedulerAddTask(robotTurnRightDouble, NULL, newDelay, 0);
			newDelay += ROBOT_TURNDOUBLE;
			break;
		case BN:
			schedulerAddTask(robotTurnBackNormal, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case BI:
			schedulerAddTask(robotTurnBackInvert, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			break;
		case B2:
			schedulerAddTask(robotTurnBackDouble, NULL, newDelay, 0);
			newDelay += ROBOT_TURNDOUBLE;
			break;
		case DN:
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotTurnFrontNormal, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case DI:
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotTurnFrontInvert, NULL, newDelay, 0);
			newDelay += ROBOT_TURNSINGLE;
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case D2:
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotTurnFrontDouble, NULL, newDelay, 0);
			newDelay += ROBOT_TURNDOUBLE;
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case xN:
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case xI:
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case x2:
			schedulerAddTask(robotFlipXDouble, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPDOUBLE;
			break;
		case yN:
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotFlipZNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case yI:
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotFlipZInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case y2:
			schedulerAddTask(robotFlipXNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			schedulerAddTask(robotFlipZDouble, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPDOUBLE;
			schedulerAddTask(robotFlipXInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case zN:
			schedulerAddTask(robotFlipZNormal, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case zI:
			schedulerAddTask(robotFlipZInvert, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPSINGLE;
			break;
		case z2:
			schedulerAddTask(robotFlipZDouble, NULL, newDelay, 0);
			newDelay += ROBOT_FLIPDOUBLE;
			break;
		default:
			break;
		}
	}
}
