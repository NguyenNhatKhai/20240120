/*
 * scheduler.c
 *
 *  Created on: Jan 15, 2024
 *      Author: Nhat Khai
 */

#include "scheduler.h"

static schedulerTask tasks[SCHEDULER_NUMBER];

void schedulerCheck(void *arguments) {
	HAL_GPIO_TogglePin(SCHEDULER_LED_GPIO_Port, SCHEDULER_LED_Pin);
}

void schedulerInit(void) {
    for (uint8_t i = 0; i < SCHEDULER_NUMBER; i ++) {
        tasks[i].function = 0;
        tasks[i].arguments = 0;
        tasks[i].id = SCHEDULER_NUMBER - i;
        tasks[i].delay = 0;
        tasks[i].period = 0;
        tasks[i].flag = 0;
    }
    schedulerAddTask(schedulerCheck, NULL, 0, 1000);
}

void schedulerUpdate(void) {
    if (tasks[0].function == 0) return;
	if (tasks[0].delay > 0) {
		if (tasks[0].delay > SCHEDULER_TICK) {
			tasks[0].delay -= SCHEDULER_TICK;
		}
		else {
			tasks[0].delay = 0;
		}
	}
	if (tasks[0].delay == 0) {
		tasks[0].flag = 1;
	}
}

void schedulerDispatch(void) {
    if (tasks[0].flag == 0) return;
    (*tasks[0].function)(tasks[0].arguments);
    if (tasks[0].period > 0) {
    	schedulerRefreshTask();
    }
    else {
    	schedulerDeleteTask(tasks[0].id);
    }
}

uint8_t schedulerAddTask(void (*function)(void*), void *arguments, uint32_t delay, uint32_t period) {
    if (tasks[SCHEDULER_NUMBER - 1].function != 0) return 0;
    uint8_t currentID = tasks[SCHEDULER_NUMBER - 1].id;
    uint32_t currentDelay = 0;
    for (uint8_t i = 0; i < SCHEDULER_NUMBER; i ++) {
        currentDelay += tasks[i].delay;
        if (currentDelay > delay || tasks[i].function == 0) {
            for (uint8_t j = SCHEDULER_NUMBER - 1; j > i; j --) {
                tasks[j] = tasks[j - 1];
            }
            tasks[i].function = function;
            tasks[i].arguments = arguments;
            tasks[i].id = currentID;
            tasks[i].period = period;
            tasks[i].flag = 0;
            if (currentDelay > delay) {
                int newDelay = currentDelay - delay;
                tasks[i].delay = tasks[i + 1].delay - newDelay;
                if (tasks[i].delay == 0) {
                    tasks[i].flag = 1;
                }
                tasks[i + 1].delay = newDelay;
                if (tasks[i + 1].delay == 0) {
                    tasks[i + 1].flag = 1;
                }
            }
            else {
                tasks[i].delay = delay - currentDelay;
                if (tasks[i].delay == 0) {
                    tasks[i].flag = 1;
                }
            }
            return tasks[i].id;
        }
    }
    return 0;
}

unsigned char schedulerDeleteTask(uint8_t id) {
    for (uint8_t i = 0; i < SCHEDULER_NUMBER; i ++) {
    	if (tasks[i].function == 0) return 0;
        if (tasks[i].id == id) {
            uint8_t currentID = tasks[i].id;
            if (tasks[i + 1].function != 0) {
                tasks[i + 1].delay += tasks[i].delay;
            }
            for (uint8_t j = i; j < SCHEDULER_NUMBER - 1; j ++) {
                tasks[j] = tasks[j + 1];
            }
            tasks[SCHEDULER_NUMBER - 1].function = 0;
            tasks[SCHEDULER_NUMBER - 1].arguments = 0;
            tasks[SCHEDULER_NUMBER - 1].id = currentID;
            tasks[SCHEDULER_NUMBER - 1].delay = 0;
            tasks[SCHEDULER_NUMBER - 1].period = 0;
            tasks[SCHEDULER_NUMBER - 1].flag = 0;
            return 1;
        }
    }
    return 0;
}

unsigned char schedulerRefreshTask(void) {
    if (tasks[0].function == 0) return 0;
    schedulerTask currentTask = tasks[0];
    uint32_t currentDelay = 0;
    for (uint8_t i = 0; i < SCHEDULER_NUMBER; i ++) {
        if (i + 1 == SCHEDULER_NUMBER || tasks[i + 1].function == NULL) {
            tasks[i].function = currentTask.function;
            tasks[i].arguments = currentTask.arguments;
            tasks[i].id = currentTask.id;
            tasks[i].period = currentTask.period;
            tasks[i].flag = 0;
            tasks[i].delay = currentTask.period - currentDelay;
            if (tasks[i].delay == 0) {
                tasks[i].flag = 1;
            }
            return 1;
        }
        currentDelay += tasks[i + 1].delay;
        if (currentDelay > currentTask.period) {
            tasks[i].function = currentTask.function;
            tasks[i].arguments = currentTask.arguments;
            tasks[i].id = currentTask.id;
            tasks[i].period = currentTask.period;
            tasks[i].flag = 0;
            int newDelay = currentDelay - currentTask.period;
            tasks[i].delay = tasks[i + 1].delay - newDelay;
            if (tasks[i].delay == 0) {
                tasks[i].flag = 1;
            }
            tasks[i + 1].delay -= tasks[i].delay;
            if (tasks[i + 1].delay == 0) {
                tasks[i + 1].flag = 1;
            }
            return 1;
        }
        else {
            tasks[i] = tasks[i + 1];
        }
    }
    return 0;
}
