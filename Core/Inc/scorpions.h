/*
 * scorpions.h
 *
 *  Created on: Jan 23, 2024
 *      Author: Nhat Khai
 */

#ifndef INC_SCORPIONS_H_
#define INC_SCORPIONS_H_

#include "main.h"
#include "scheduler.h"
#include "robot.h"

typedef enum {
	UN, UI, U2,
	LN, LI, L2,
	FN, FI, F2,
	RN, RI, R2,
	BN, BI, B2,
	DN, DI, D2,
	xN, xI, x2,
	yN, yI, y2,
	zN, zI, z2
} scorpionsMove;

void scorpionsScramble(void *);

#endif /* INC_SCORPIONS_H_ */
