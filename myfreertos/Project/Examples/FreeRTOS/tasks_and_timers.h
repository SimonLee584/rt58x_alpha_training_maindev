#ifndef TASKS_AND_TIMERS_H
#define TASKS_AND_TIMERS_H

#include "FreeRTOS.h"
#include "task.h"

/* 创建两个任务和两个定时器的初始化接口 */
void TasksAndTimers_Task(void *parameters_ptr);

#endif /* TASKS_AND_TIMERS_H */
