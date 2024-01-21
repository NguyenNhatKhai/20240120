/*
 * scheduler.h
 *
 *  Created on: Jan 15, 2024
 *      Author: Nhat Khai
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include <stdint.h>
#include <stdio.h>
#include "main.h"

#define SCHEDULER_NUMBER 32
#define SCHEDULER_TICK 1				// in milliseconds

typedef struct {
    void (*function)(void *);
    void *arguments;
    uint8_t id;
    uint32_t delay;
    uint32_t period;
    unsigned char flag;
} schedulerTask;

void schedulerInit(void);
void schedulerUpdate(void);
void schedulerDispatch(void);

uint8_t schedulerAddTask(void (*function)(void *), void *arguments, uint32_t delay, uint32_t period);
unsigned char schedulerDeleteTask(uint8_t id);
unsigned char schedulerRefreshTask(void);

#endif /* INC_SCHEDULER_H_ */
